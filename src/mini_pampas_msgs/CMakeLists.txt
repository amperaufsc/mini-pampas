cmake_minimum_required(VERSION 3.8)
project(mini_pampas_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  "msg/DriveCommand.msg"
  "msg/WheelSpeed.msg"
  DEPENDENCIES std_msgs
)

ament_package()
