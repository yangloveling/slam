cmake_minimum_required(VERSION 3.8)
project(fastlio2_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

find_package(OpenMP)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp
  ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_build
)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp/include
)

add_executable(relocalization_node src/relocalization_node.cpp)

target_link_libraries(relocalization_node
  ${PCL_LIBRARIES}
  small_gicp
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(relocalization_node OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(relocalization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

install(TARGETS
  relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

