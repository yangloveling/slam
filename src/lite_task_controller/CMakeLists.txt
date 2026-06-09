cmake_minimum_required(VERSION 3.8)
project(lite_task_controller)

# ============================================================================
# C++ 标准
# ============================================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# 编译选项
# ============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================================
# 查找依赖
# ============================================================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(robot_common REQUIRED)

# ============================================================================
# 生成 service 接口
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetRelativeGoal.srv"
  "srv/SetAbsoluteGoal.srv"
  DEPENDENCIES
)

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(
  include
)

# ============================================================================
# lite_task_controller_node
# ============================================================================
add_executable(lite_task_controller_node
  src/lite_task_controller_node.cpp
)

ament_target_dependencies(lite_task_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  robot_common
)

# 让节点链接本包生成的 srv 接口
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(lite_task_controller_node
  "${cpp_typesupport_target}"
)

target_include_directories(lite_task_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================================
# 安装可执行文件
# ============================================================================
install(TARGETS
  lite_task_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# 安装头文件
# ============================================================================
install(DIRECTORY
  include/
  DESTINATION include/
  OPTIONAL
)

# ============================================================================
# 安装 srv
# ============================================================================
install(DIRECTORY
  srv/
  DESTINATION share/${PROJECT_NAME}/srv
  OPTIONAL
)

# ============================================================================
# 安装 launch
# ============================================================================
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# ============================================================================
# 安装 config
# ============================================================================
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ============================================================================
# 导出依赖
# ============================================================================
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  rosidl_default_runtime
  robot_common
)

# ============================================================================
# package
# ============================================================================
ament_package()
