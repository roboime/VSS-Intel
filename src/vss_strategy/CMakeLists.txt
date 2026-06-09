cmake_minimum_required(VERSION 3.16)
project(vss_strategy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vss_msgs REQUIRED)
find_package(vss_core REQUIRED)

add_executable(strategy_node src/strategy_node.cpp)

ament_target_dependencies(strategy_node rclcpp vss_msgs vss_core)

target_include_directories(strategy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${vss_core_INCLUDE_DIRS}
)

install(TARGETS strategy_node DESTINATION lib/${PROJECT_NAME})

ament_package()
