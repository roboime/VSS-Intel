cmake_minimum_required(VERSION 3.16)
project(vss_bringup)

find_package(ament_cmake REQUIRED)

# Instala as pastas launch e config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
