#include <portaudio.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include "turtleaudio/msg/stereo_audio_block.hpp"
#include <time.h>
#include "std_msgs/msg/header.hpp"
#include "rclcpp/qos.hpp"

#define CHANNELS 2 // Stéréo
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 1024

#define MAX_FRAMES 2000 // Nombre maximum de frames à lire
using namespace std::chrono_literals;
using turtleaudio::msg::StereoAudioBlock;

PaStream *_init_input_stream()
{
  PaStream *stream;
  PaError err;
  err = Pa_Initialize();
  if (err != paNoError)
  {
    std::cerr << "Erreur d'initialisation PortAudio : " << Pa_GetErrorText(err) << std::endl;
    exit(err);
  }

  err = Pa_OpenDefaultStream(&stream,
                             CHANNELS, // 2 canaux d'entrée (stéréo)
                             0,        // Pas de sortie
                             paInt16,
                             SAMPLE_RATE,
                             BUFFER_SIZE,
                             NULL,
                             NULL);
  if (err != paNoError)
  {
    std::cerr << "Erreur d'ouverture du flux : " << Pa_GetErrorText(err) << std::endl;
    exit(err);
  }

  err = Pa_StartStream(stream);
  if (err != paNoError)
  {
    std::cerr << "Erreur au démarrage du flux : " << Pa_GetErrorText(err) << std::endl;
    exit(err);
  }

  return stream;
}
class AudioRecorder : public rclcpp::Node
{
public:
  AudioRecorder()
      : Node("audio_recorder")
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
    timer = this->create_wall_timer(
        period, std::bind(&AudioRecorder::get_sample, this));

    publisher_sin = this->create_publisher<StereoAudioBlock>("audio_sin", qos_audio_realtime);

    msg_sin.left.data.resize(BUFFER_SIZE);
    msg_sin.right.data.resize(BUFFER_SIZE);

    timer_sin = this->create_wall_timer(
        period, std::bind(&AudioRecorder::get_sample_sin, this));
  }

private:

void get_sample_sin()
{
  constexpr double frequency = 440.0; 
  constexpr double amplitude = 300.0; 
  constexpr double phase_shift = M_PI / 2;

  for (size_t i = 0; i < BUFFER_SIZE; ++i)
  {
    double t = static_cast<double>(sample_index_sin++) / SAMPLE_RATE;

    int16_t left_sample = static_cast<int16_t>(amplitude * sin(2 * M_PI * frequency * t));
    int16_t right_sample = static_cast<int16_t>(amplitude * sin(2 * M_PI * frequency * t + phase_shift));

    msg_sin.left.data[i] = left_sample;
    msg_sin.right.data[i] = right_sample;
  }

  msg_sin.header.frame_id = std::to_string(frame_count_sin);
  msg_sin.header.stamp = this->now();
  frame_count_sin++;

  publisher_sin->publish(msg_sin);
}
  StereoAudioBlock msg_sin;
  rclcpp::Publisher<StereoAudioBlock>::SharedPtr publisher_sin;
  rclcpp::TimerBase::SharedPtr timer_sin;
  int64_t sample_index_sin = 0;  
  int frame_count_sin = 0;

  void get_sample()
  {
    static PaStream *stream = _init_input_stream();

    PaError err = Pa_ReadStream(stream, buffer.data(), BUFFER_SIZE);
    if (err != paNoError)
    {
      std::cerr << "Erreur de lecture : " << Pa_GetErrorText(err) << std::endl;
      return;
    }

    const int16_t *src = buffer.data();
    short int *dst_left = msg.left.data.data();
    short int *dst_right = msg.right.data.data();

    for (size_t i = 0; i < BUFFER_SIZE; ++i)
    {
      *dst_left++ = static_cast<int16_t>(*src++);  // canal gauche
      *dst_right++ = static_cast<int16_t>(*src++); // canal droit
    }

    msg.header.frame_id = std::to_string(frame_count);
    msg.header.stamp = this->now();
    publisher->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Frame %d", frame_count);
    if (frame_count >= MAX_FRAMES)
    {
      RCLCPP_INFO(this->get_logger(), "Nombre maximum de frames atteint, arrêt du noeud.");
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
  int frame_count = 0;
};

// Main entry point
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioRecorder>());
  rclcpp::shutdown();
  return 0;
}