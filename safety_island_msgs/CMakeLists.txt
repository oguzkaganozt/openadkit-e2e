cmake_minimum_required(VERSION 3.16)
project(safety_island_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(autoware_control_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ApprovedRequest.msg"
  DEPENDENCIES builtin_interfaces autoware_control_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces
  autoware_control_msgs)

ament_package()
