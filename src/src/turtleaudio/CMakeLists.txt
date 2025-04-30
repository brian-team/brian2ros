cmake_minimum_required(VERSION 3.8)
project(turtleaudio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(audio_publisher src/audio_publisher.cpp)
ament_target_dependencies(audio_publisher rclcpp std_msgs)

install(TARGETS
  audio_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
