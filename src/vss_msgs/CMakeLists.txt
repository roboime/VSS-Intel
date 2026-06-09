cmake_minimum_required(VERSION 3.16)
project(vss_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallState.msg"
  "msg/GameState.msg"
  "msg/RobotCommand.msg"
  "msg/RobotCommandArray.msg"
  "msg/RobotState.msg"
  DEPENDENCIES std_msgs
)

ament_package()
