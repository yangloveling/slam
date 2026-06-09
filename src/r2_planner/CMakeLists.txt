cmake_minimum_required(VERSION 3.8)
project(r2_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_common REQUIRED)

add_executable(r2_dijkstra_planner_node src/r2_dijkstra_planner_node.cpp)

ament_target_dependencies(
  r2_dijkstra_planner_node
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  robot_common
)

install(TARGETS
  r2_dijkstra_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(r2_decision_node src/r2_decision_node.cpp)

ament_target_dependencies(
  r2_decision_node
  rclcpp
  std_msgs
)

install(TARGETS
  r2_decision_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(serial_reader_node src/serial_reader_node.cpp)
ament_target_dependencies(serial_reader_node rclcpp std_msgs)

install(TARGETS serial_reader_node DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
