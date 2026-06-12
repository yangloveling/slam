from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('match_state_machine')
    config_file = os.path.join(pkg_share, 'config', 'zone1_runner.yaml')

    return LaunchDescription([
        Node(
            package='match_state_machine',
            executable='zone1_runner',
            name='zone1_runner',
            output='screen',
            parameters=[config_file]
        )
    ])

