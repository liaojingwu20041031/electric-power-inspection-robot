cmake_minimum_required(VERSION 3.8)
project(ylhb_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskEvent.msg"
  "msg/TaskStatus.msg"
  "msg/SayText.msg"
  "msg/VoiceStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
