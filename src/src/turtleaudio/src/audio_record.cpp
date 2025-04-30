#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <portaudio.h>
#include <vector>
#include <cstring>

#define SAMPLE_RATE 48000
#define FRAMES_PER_BUFFER 512
#define NUM_CHANNELS 2

class AudioPublisher : public rclcpp::Node
{
public:
  AudioPublisher()
  : Node("audio_publisher")
  {
    // Create a ROS 2 publisher for audio data
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("audio_raw", 10);

    // Initialize PortAudio library
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize PortAudio: %s", Pa_GetErrorText(err));
      rclcpp::shutdown();
      return;
    }

    // Set input device parameters
    input_params_.device = Pa_GetDefaultInputDevice();
    if (input_params_.device == paNoDevice) {
      RCLCPP_FATAL(this->get_logger(), "No default input device found.");
      Pa_Terminate();
      rclcpp::shutdown();
      return;
    }

    input_params_.channelCount = NUM_CHANNELS;
    input_params_.sampleFormat = paInt16; // 16-bit audio
    input_params_.suggestedLatency = Pa_GetDeviceInfo(input_params_.device)->defaultLowInputLatency;
    input_params_.hostApiSpecificStreamInfo = nullptr;

    // Open audio input stream
    err = Pa_OpenStream(
      &stream_,
      &input_params_,
      nullptr,              // no output
      SAMPLE_RATE,
      FRAMES_PER_BUFFER,
      paClipOff,
      nullptr,
      nullptr
    );
    if (err != paNoError) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open PortAudio stream: %s", Pa_GetErrorText(err));
      Pa_Terminate();
      rclcpp::shutdown();
      return;
    }

    // Start audio stream
    err = Pa_StartStream(stream_);
    if (err != paNoError) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start PortAudio stream: %s", Pa_GetErrorText(err));
      Pa_CloseStream(stream_);
      Pa_Terminate();
      rclcpp::shutdown();
      return;
    }

    // Setup timer to capture and publish audio every 50ms
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AudioPublisher::capture_and_publish, this)
    );
  }

  ~AudioPublisher()
  {
    Pa_StopStream(stream_);
    Pa_CloseStream(stream_);
    Pa_Terminate();
  }

private:
  void capture_and_publish()
  {
    // Capture buffer (stereo: 2 channels, int16 = 2 bytes/sample)
    std::vector<int16_t> buffer(FRAMES_PER_BUFFER * NUM_CHANNELS);
    PaError err = Pa_ReadStream(stream_, buffer.data(), FRAMES_PER_BUFFER);

    if (err != paNoError) {
      RCLCPP_WARN(this->get_logger(), "Audio read error: %s", Pa_GetErrorText(err));
      return;
    }

    // Create ROS message (convert int16 to byte array)
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(buffer.size() * sizeof(int16_t));
    std::memcpy(msg.data.data(), buffer.data(), msg.data.size());

    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  PaStream *stream_;
  PaStreamParameters input_params_;
};

// Main entry point
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioPublisher>());
  rclcpp::shutdown();
  return 0;
}
