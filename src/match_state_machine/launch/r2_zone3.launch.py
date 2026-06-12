#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'match_state_machine'

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'r2_zone3.yaml'
    )

    r2_zone3_node = Node(
        package=package_name,
        executable='r2_zone3_node',
        name='r2_zone3_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                # 这里做一层保险覆盖，避免 yaml 没更新时仍用旧参数
                'odom_topic': '/odometry/filtered',
                'cmd_vel_topic': '/cmd_vel',
                'absolute_goal_service': '/set_absolute_goal',
                'use_lite_absolute_goal': True,
                'can_tx_topic': '/can_tx',
            }
        ]
    )

    return LaunchDescription([
        r2_zone3_node
    ])

