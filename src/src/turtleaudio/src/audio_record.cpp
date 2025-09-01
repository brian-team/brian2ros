#include <portaudio.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include "turtleaudio/msg/stereo_audio_block.hpp"
#include <time.h>
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include <sndfile.h>

#define CHANNELS 2 // Stéréo
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 1024

using namespace std::chrono_literals;
using turtleaudio::msg::StereoAudioBlock;
using std::placeholders::_1;

PaStream *_init_input_stream()
{
  PaStream *stream;
  PaError err;
  err = Pa_Initialize();
  if (err != paNoError)
  {
    std::cerr << "Initialization error of PortAudio : " << Pa_GetErrorText(err) << std::endl;
    exit(err);
  }

  err = Pa_OpenDefaultStream(&stream,
                             CHANNELS,
                             0,
                             paInt16,
                             SAMPLE_RATE,
                             BUFFER_SIZE,
                             NULL,
                             NULL);
  if (err != paNoError)
  {
    std::cerr << "Stream open Error : " << Pa_GetErrorText(err) << std::endl;
    exit(err);
  }

  err = Pa_StartStream(stream);
  if (err != paNoError)
  {
    std::cerr << "Stream Error in start : " << Pa_GetErrorText(err) << std::endl;
    exit(err);
  }

  return stream;
}

class AudioRecorder : public rclcpp::Node
{
public:
AudioRecorder(const std::string &wav_path = "", int max_frames = 20000)
    : Node("audio_recorder"), wav_path_(wav_path), max_frames_(max_frames)
  {
    rclcpp::QoS qos_audio_realtime(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_audio_realtime
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        .keep_last(5);

    publisher = this->create_publisher<StereoAudioBlock>("audio_data", qos_audio_realtime);

    msg.left.data.resize(BUFFER_SIZE);
    msg.right.data.resize(BUFFER_SIZE);
    buffer.resize(BUFFER_SIZE * CHANNELS);

    auto seconds = static_cast<double>(BUFFER_SIZE) / SAMPLE_RATE;
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(seconds));


    if (wav_path_ == "sin") 
    {
      RCLCPP_INFO(this->get_logger(), "Sinusoidal audio input for testing.");
      timer = this->create_wall_timer(
          period, std::bind(&AudioRecorder::get_sample_sin, this));
    }
    else if (wav_path_ == "portaudio")
    {
      RCLCPP_INFO(this->get_logger(), "Real-time audio input with PortAudio.");
      timer = this->create_wall_timer(
          period, std::bind(&AudioRecorder::get_sample, this));
    }
    else if (wav_path_ == "sin_gz")
    {
      pos_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos_audio_realtime, std::bind(&AudioRecorder::odom_callback, this, _1)); 

      RCLCPP_INFO(this->get_logger(), "Sinusoidal audio input for testing with gz.");
      timer_sin = this->create_wall_timer(
          period, std::bind(&AudioRecorder::get_sample_sin, this));
    }    
    else if (!wav_path_.empty()) 
    {
      RCLCPP_INFO(this->get_logger(), "Read and publish WAV file : %s", wav_path_.c_str());
      timer = this->create_wall_timer(
          period, std::bind(&AudioRecorder::get_sample_from_wav, this));
      open_wav_file();
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "No valid input source specified. Use a WAV file path or 'sin' for sinusoidal input or 'portaudio' for real-time audio input.");
      rclcpp::shutdown();
    }

  }

  ~AudioRecorder() 
  {
    if (sndfile_) 
    {
      sf_close(sndfile_);
      sndfile_ = nullptr;
    }
  }

private:

  // This function opens a WAV file using the libsndfile library.
  // It initializes the SF_INFO structure and opens the file for reading.
  // If the file cannot be opened or has an unsupported number of channels,
  // it logs an error message and shuts down the ROS node.

  void open_wav_file() 
  {
    sfinfo_ = {};
    // Base directory where WAV files are stored
    std::string base_path = "src/src/turtleaudio/src/recorded_sound/";
    // Concatenate full path
    std::string full_path = base_path + wav_path_;

    sndfile_ = sf_open(full_path.c_str(), SFM_READ, &sfinfo_);
    if (!sndfile_) {
      RCLCPP_ERROR(this->get_logger(), "Error opening WAV file : %s", sf_strerror(nullptr));
      rclcpp::shutdown();
    }
    if (sfinfo_.channels < 1 || sfinfo_.channels > 2) {
      RCLCPP_ERROR(this->get_logger(), "Unsupported number of channels in WAV file: %d", sfinfo_.channels);
      rclcpp::shutdown();
    }
  }

  void get_sample_from_wav() 
  {
    //RCLCPP_INFO(this->get_logger(), "Reading WAV file: %d", sfinfo_.channels);
    std::vector<float> read_buffer(BUFFER_SIZE * sfinfo_.channels);
    sf_count_t frames_read = sf_readf_float(sndfile_, read_buffer.data(), BUFFER_SIZE);
    if (frames_read <= 0) {
      RCLCPP_INFO(this->get_logger(), "End of WAV file reached");
      rclcpp::shutdown();
      return;
    }
    for (size_t i = 0; i < frames_read; ++i) {
      msg.left.data[i] = static_cast<int16_t>(read_buffer[i * sfinfo_.channels] * 32767.0); // Assuming 16-bit PCM
      msg.right.data[i] = static_cast<int16_t>(read_buffer[i * sfinfo_.channels + 1] * 32767.0); // Assuming 16-bit PCM
    }

    msg.header.frame_id = std::to_string(frame_count);
    msg.header.stamp = this->now();
    publisher->publish(msg);
    frame_count++;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    change_orientation = false;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    double rad_z = atan2(2*(w*z), 1 - 2*(z*z));
    orientation = source_orientation - rad_z; // Calculate the phase shift based on the orientation of the sound source
    RCLCPP_INFO(this->get_logger(), "Orientation of sound source: %f rad", rad_z * 180 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Phase shift: %f rad", orientation * 180 / M_PI);
    // Update the source orientation based on the current orientation of the sound source
  }
  double source_orientation = M_PI / 2; // Initial orientation of the sound source
  bool change_orientation = true;
  // This function generates a sine wave signal for testing purposes.
  // It simulates a sound wave with a frequency of 440 Hz (A4 note) and a phase shift.
  // The left and right channels are delayed by a small amount to simulate stereo sound.

  void get_sample_sin() 
  {
    constexpr double frequency = 800.0;
    constexpr double amplitude = 32767.0 / 2; // Max amplitude for 16-bit audio
    constexpr double sound_speed = 343.0;
    constexpr double distance = 0.2;
    constexpr double max_delay = distance / sound_speed;

    for (size_t i = 0; i < BUFFER_SIZE; ++i) 
    {
      double t = static_cast<double>(sample_index_sin++) / SAMPLE_RATE;
      double left_delay = -0.5 * max_delay * sin(orientation);
      double right_delay = 0.5 * max_delay * sin(orientation);

      int16_t left_sample = static_cast<int16_t>(amplitude * sin(2 * M_PI * frequency * (t - left_delay)));
      int16_t right_sample = static_cast<int16_t>(amplitude * sin(2 * M_PI * frequency * (t - right_delay)));

      msg.left.data[i] = left_sample;
      msg.right.data[i] = right_sample;
    }

    if (frame_count_sin % 100 == 0)
      //RCLCPP_INFO(this->get_logger(), "Frame %d", frame_count_sin);
      RCLCPP_INFO(this->get_logger(), "Phase shift in get_sample : %f", orientation * 180 / M_PI);

    if (frame_count_sin >= max_frames_)
    {
      RCLCPP_INFO(this->get_logger(), "Maximum number of frames reached, stopping node.");
      rclcpp::shutdown();
    }

    msg.header.frame_id = std::to_string(frame_count_sin);
    msg.header.stamp = this->now();
    frame_count_sin++;
    if (change_orientation)
    {
    orientation += M_PI / 250; // Increment the phase shift for the next sample
    }
    publisher->publish(msg);
  }

  // This function reads audio samples from the input stream and publishes them as StereoAudioBlock messages.
  // It uses PortAudio to read the audio data and fills the left and right channels of the message.
  // The function also manages the frame count and stops the stream after reaching the maximum number of frames.
  // If an error occurs during reading, it logs the error message and continues to the next iteration.
  // The function is called periodically based on the timer set in the constructor.

  void get_sample() 
  {
    static PaStream *stream = _init_input_stream();

    PaError err = Pa_ReadStream(stream, buffer.data(), BUFFER_SIZE);
    if (err != paNoError) 
    {
      std::cerr << "Stream read error : " << Pa_GetErrorText(err) << std::endl;
      return;
    }

    const int16_t *src = buffer.data();
    short int *dst_left = msg.left.data.data();
    short int *dst_right = msg.right.data.data();

    for (size_t i = 0; i < BUFFER_SIZE; ++i) 
    {
      *dst_left++ = *src++;
      *dst_right++ = *src++;
    }

    msg.header.frame_id = std::to_string(frame_count);
    msg.header.stamp = this->now();
    publisher->publish(msg);

    if (frame_count % 100 == 0)
      RCLCPP_INFO(this->get_logger(), "Frame %d", frame_count);

    if (frame_count >= max_frames_)
     {
      RCLCPP_INFO(this->get_logger(), "Maximum number of frames reached, stopping node.");
      Pa_StopStream(stream);
      Pa_CloseStream(stream);
      Pa_Terminate();
      rclcpp::shutdown();
    }
    frame_count++;
  }

  StereoAudioBlock msg;
  std::vector<int16_t> buffer;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<StereoAudioBlock>::SharedPtr publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_subscriber;
  int frame_count = 0;

  rclcpp::TimerBase::SharedPtr timer_sin;
  int64_t sample_index_sin = 0;
  int frame_count_sin = 0;
  double orientation = 0;

  std::string wav_path_;
  SNDFILE *sndfile_ = nullptr;
  SF_INFO sfinfo_ = {};
  int max_frames_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string wav_file = (argc > 1) ? argv[1] : "";
  int max_frames = (argc > 2) ? std::atoi(argv[2]) : 20000;

  rclcpp::spin(std::make_shared<AudioRecorder>(wav_file, max_frames));
  rclcpp::shutdown();
  return 0;
}
