from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('r2_planner')
    config_file = os.path.join(pkg_share, 'config', 'r2_astar_planner.yaml')

    return LaunchDescription([
        Node(
            package='r2_planner',
            executable='r2_astar_planner_node',
            name='r2_astar_planner_node',
            output='screen',
            parameters=[config_file]
        )
    ])
