cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(odom_to_yaw src/odom_to_yaw.cpp)
ament_target_dependencies(odom_to_yaw
  rclcpp
  nav_msgs
  std_msgs
  tf2
)

add_executable(controller_node src/omni_nav_controller.cpp)
ament_target_dependencies(controller_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_include_directories(bridge_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  odom_to_yaw
  controller_node
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
