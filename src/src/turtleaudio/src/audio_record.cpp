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

#define CHANNELS 2       // Stéréo
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 1024 

#define MAX_FRAMES 500 // Nombre maximum de frames à lire
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
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
        .keep_last(1);

      publisher = this->create_publisher<StereoAudioBlock>("audio_data", qos_audio_realtime);
      buffer.resize(BUFFER_SIZE * CHANNELS);
      auto seconds = static_cast<double>(BUFFER_SIZE) / SAMPLE_RATE;
      auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(seconds));
      timer = this->create_wall_timer(
       period, std::bind(&AudioRecorder::get_sample, this));
      
      publisher_sin = this->create_publisher<StereoAudioBlock>("audio_sin", 10);
      timer_sin = this->create_wall_timer(
       period, std::bind(&AudioRecorder::get_sample_sin, this));
    }
  private:
    void get_sample_sin()
    {
      StereoAudioBlock msg;
      msg.left.data.resize(BUFFER_SIZE);
      msg.right.data.resize(BUFFER_SIZE);
      for (size_t i = 0; i < BUFFER_SIZE; ++i)
      {
          msg.left.data[i] = static_cast<int>(3000 * sin((2 * M_PI * 440 * i) / SAMPLE_RATE));
          msg.right.data[i] = static_cast<int>(3000 * sin((2 * M_PI * 440 * i) / SAMPLE_RATE));
      }
      msg.header.frame_id = std::to_string(frame_count);
      msg.header.stamp = this->now();
      frame_count_sin++;
      publisher_sin->publish(msg);
    }
    rclcpp::Publisher<StereoAudioBlock>::SharedPtr publisher_sin;
    rclcpp::TimerBase::SharedPtr timer_sin;
    int frame_count_sin = 0;
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
          msg.left.data[i] = static_cast<int>(buffer[2 * i]);
          msg.right.data[i] = static_cast<int>(buffer[2 * i + 1]);
      }
      msg.header.frame_id = std::to_string(frame_count);
      msg.header.stamp = this->now();
      frame_count++;
      RCLCPP_INFO(this->get_logger(), "Frame %d", frame_count);
      if (frame_count >= MAX_FRAMES)
      {
          RCLCPP_INFO(this->get_logger(), "Nombre maximum de frames atteint, arrêt du noeud.");
          rclcpp::shutdown(); 
          Pa_Terminate();
      }
      publisher->publish(msg);
    }
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