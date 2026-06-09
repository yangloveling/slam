from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_output = LaunchConfiguration('control_output')
    log_level = LaunchConfiguration('log_level')
    debug_print_every_ms = LaunchConfiguration('debug_print_every_ms')
    controller_config = PathJoinSubstitution([
        FindPackageShare('lite_task_controller'),
        'config',
        'controller.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_output',
            default_value='screen',
            description='Output mode for task controller node: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='ROS log level for bringup nodes'
        ),
        DeclareLaunchArgument(
            'debug_print_every_ms',
            default_value='1000',
            description='Throttle period for task controller debug logs in milliseconds'
        ),
        Node(
            package='lite_task_controller',
            executable='lite_task_controller_node',
            name='lite_task_controller',
            output=control_output,
            parameters=[
                controller_config,
                {'debug_print_every_ms': debug_print_every_ms},
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )
    ])
