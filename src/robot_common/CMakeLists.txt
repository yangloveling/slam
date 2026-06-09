cmake_minimum_required(VERSION 3.8)
project(robot_common)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
