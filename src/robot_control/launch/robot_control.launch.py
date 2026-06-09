from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    control_output = LaunchConfiguration('control_output')
    log_level = LaunchConfiguration('log_level')
    debug_print_every_ms = LaunchConfiguration('debug_print_every_ms')

    omni_config = PathJoinSubstitution([
        FindPackageShare('robot_control'),
        'config',
        'omni_nav_controller.yaml'
    ])

    # imu_odom_bridge
    # 不再通过 launch 传入 input_topic/output_topic/tx/ty/tz/roll/pitch/yaw 等参数
    # 节点会使用 bridge_node.cpp 里面 declare_parameter 的默认值
    imu_odom_bridge = Node(
        package='robot_control',
        executable='bridge_node',
        name='imu_odom_bridge',
        output=control_output,
        arguments=['--ros-args', '--log-level', log_level],
    )

    # odom_to_yaw node
    odom_to_yaw_node = Node(
        package='robot_control',
        executable='odom_to_yaw',
        name='odom_to_yaw',
        output=control_output,
        arguments=['--ros-args', '--log-level', log_level],
    )

    # omni_nav_controller
    omni_nav_controller = Node(
        package='robot_control',
        executable='controller_node',
        name='omni_nav_controller',
        output=control_output,
        parameters=[
            omni_config,
            {'debug_print_every_ms': debug_print_every_ms},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_output',
            default_value='screen',
            description='Output mode for control nodes: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='ROS log level for bringup nodes'
        ),
        DeclareLaunchArgument(
            'debug_print_every_ms',
            default_value='1000',
            description='Throttle period for controller debug logs in milliseconds'
        ),
        imu_odom_bridge,
        odom_to_yaw_node,
        omni_nav_controller,
    ])
