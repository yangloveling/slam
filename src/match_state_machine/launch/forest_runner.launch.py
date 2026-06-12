from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'match_state_machine'

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'forest_runner.yaml'
    )

    return LaunchDescription([
        LogInfo(
            msg=[
                '[forest_runner] loading config: ',
                config_file
            ]
        ),

        Node(
            package=package_name,
            executable='forest_runner_node',
            name='forest_runner',
            output='screen',
            parameters=[config_file],
            emulate_tty=True
        )
    ])

