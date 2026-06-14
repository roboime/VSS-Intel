cmake_minimum_required(VERSION 3.16)
project(vss_core)
 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
find_package(ament_cmake REQUIRED)
 
add_library(vss_core INTERFACE)
 
target_include_directories(vss_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
 
install(TARGETS vss_core EXPORT vss_core_targets)
install(DIRECTORY include/ DESTINATION include)
 
ament_export_targets(vss_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
 

