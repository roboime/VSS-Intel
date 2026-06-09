cmake_minimum_required(VERSION 3.16)
project(vss_fira_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vss_msgs REQUIRED)
find_package(vss_core REQUIRED)
find_package(Protobuf REQUIRED)

add_executable(fira_bridge_node
  src/fira_bridge_node.cpp
  include/proto/packet.pb.cc
  include/proto/command.pb.cc
  include/proto/common.pb.cc
  include/proto/replacement.pb.cc
)

ament_target_dependencies(fira_bridge_node rclcpp vss_msgs vss_core)

target_include_directories(fira_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${vss_core_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(fira_bridge_node
  ${Protobuf_LIBRARIES}
)

install(TARGETS fira_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
