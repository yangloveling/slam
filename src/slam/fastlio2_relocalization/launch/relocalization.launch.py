import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('fastlio2_relocalization')
    config_file = os.path.join(pkg_dir, 'config', 'relocalization.yaml')
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    system_library_path = '/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu'
    relocalization_library_path = (
        f'{system_library_path}:{ld_library_path}'
        if ld_library_path else system_library_path
    )

    sensor_output = LaunchConfiguration('sensor_output')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_output',
            default_value='log',
            description='Output mode for relocalization node: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='ROS log level for bringup nodes'
        ),
        Node(
            package='fastlio2_relocalization',
            executable='relocalization_node',
            name='fastlio2_relocalization_node',
            output=sensor_output,
            parameters=[config_file],
            additional_env={'LD_LIBRARY_PATH': relocalization_library_path},
            arguments=['--ros-args', '--log-level', log_level],
        )
    ])
