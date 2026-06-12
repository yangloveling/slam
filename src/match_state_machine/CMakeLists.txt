cmake_minimum_required(VERSION 3.8)
project(match_state_machine)

# ============================================================================
# C++ 标准
# ============================================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================================
# 查找依赖包
# ============================================================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(lite_task_controller REQUIRED)

# ============================================================================
# 头文件目录
# ============================================================================
include_directories(
  include
)

# ============================================================================
# 1. 二区跑点节点 forest_runner_node
#    如果 src/forest_runner_node.cpp 存在才编译
# ============================================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/forest_runner_node.cpp")
  add_executable(forest_runner_node
    src/forest_runner_node.cpp
  )

  ament_target_dependencies(forest_runner_node
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    lite_task_controller
  )

  target_include_directories(forest_runner_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  install(TARGETS
    forest_runner_node
    DESTINATION lib/${PROJECT_NAME}
  )

  message(STATUS "Build target enabled: forest_runner_node")
else()
  message(STATUS "Skip target: forest_runner_node, source file not found")
endif()

# ============================================================================
# 2. 一区跑点节点 zone1_runner
#    如果 src/zone1_runner.cpp 存在才编译
# ============================================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/zone1_runner.cpp")
  add_executable(zone1_runner
    src/zone1_runner.cpp
  )

  ament_target_dependencies(zone1_runner
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    lite_task_controller
  )

  target_include_directories(zone1_runner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  install(TARGETS
    zone1_runner
    DESTINATION lib/${PROJECT_NAME}
  )

  message(STATUS "Build target enabled: zone1_runner")
else()
  message(STATUS "Skip target: zone1_runner, source file not found")
endif()

# ============================================================================
# 3. R2 三区九宫格任务节点 r2_zone3_node
#
# 当前版本：
#   1. 使用 nav_msgs/msg/Odometry 订阅 /odometry/filtered
#   2. 调用 lite_task_controller/srv/SetAbsoluteGoal 服务
#   3. 不依赖 can_msgs
#   4. 发布 std_msgs/msg/UInt32MultiArray 到 /can_tx
#
# /can_tx 数据格式：
#   data[0] = CAN ID
#   data[1] = DLC
#   data[2] = DATA[0]
#
# 例如：
#   0x88 / 0x02 -> data: [136, 1, 2]
#   0x66 / 0x03 -> data: [102, 1, 3]
# ============================================================================
add_executable(r2_zone3_node
  src/r2_zone3_node.cpp
)

ament_target_dependencies(r2_zone3_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  lite_task_controller
)

target_include_directories(r2_zone3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  r2_zone3_node
  DESTINATION lib/${PROJECT_NAME}
)

message(STATUS "Build target enabled: r2_zone3_node")

# ============================================================================
# 安装头文件
# ============================================================================
install(DIRECTORY
  include/
  DESTINATION include
  OPTIONAL
)

# ============================================================================
# 安装配置文件
# ============================================================================
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ============================================================================
# 安装 launch 文件
# ============================================================================
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# ============================================================================
# 导出 include 目录和依赖
# ============================================================================
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  lite_task_controller
)

# ============================================================================
# 导出包
# ============================================================================
ament_package()

