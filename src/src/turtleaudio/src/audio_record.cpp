#include <portaudio.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "turtleaudio/msg/stereo_audio_block.hpp"
#include <time.h>
#include "std_msgs/msg/header.hpp"

#define CHANNELS 2       // Stéréo
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 256 

using namespace std::chrono_literals;
using turtleaudio::msg::StereoAudioBlock;

PaStream* _init_input_stream()
{
    PaStream* stream;
    PaError err;

    err = Pa_Initialize();
    if (err != paNoError)
    {
        std::cerr << "Erreur d'initialisation PortAudio : " << Pa_GetErrorText(err) << std::endl;
        exit(err);
    }

    err = Pa_OpenDefaultStream(&stream,
                               CHANNELS,     // 2 canaux d'entrée (stéréo)
                               0,            // Pas de sortie
                               paFloat32,
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
      publisher = this->create_publisher<StereoAudioBlock>("audio_data", 10);
      buffer.resize(BUFFER_SIZE * CHANNELS);
      timer = this->create_wall_timer(
       (BUFFER_SIZE / SAMPLE_RATE)ms, std::bind(&AudioRecorder::get_sample, this));
    }
  private:
    void get_sample() 
    {        
      static PaStream* stream = _init_input_stream();
      PaError err = Pa_ReadStream(stream, buffer.data(), BUFFER_SIZE);
      if (err != paNoError)
      {
          std::cerr << "Erreur de lecture : " << Pa_GetErrorText(err) << std::endl;
          return;
      }

      StereoAudioBlock msg;
      msg.left.data.resize(BUFFER_SIZE);
      msg.right.data.resize(BUFFER_SIZE);

      for (size_t i = 0; i < BUFFER_SIZE; ++i)
      {
          msg.left.data[i] = static_cast<float>(buffer[2 * i]);
          msg.right.data[i] = static_cast<float>(buffer[2 * i + 1]);
      }

      msg.header.stamp = this->now();

      publisher->publish(msg);
    }
    std::vector<float> buffer;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<StereoAudioBlock>::SharedPtr publisher;
};

// Main entry point
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioRecorder>());
  rclcpp::shutdown();
  return 0;
}