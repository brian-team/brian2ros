cmake_minimum_required(VERSION 3.8)
project(turtleaudio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StereoAudioBlock.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(audio_record src/audio_record.cpp)
ament_target_dependencies(audio_record rclcpp std_msgs nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  audio_record
  DESTINATION lib/${PROJECT_NAME}
)
target_link_libraries(audio_record
  'portaudio'
  sndfile
  ${cpp_typesupport_target}
)


ament_package()
