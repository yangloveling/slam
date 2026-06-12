#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_output = LaunchConfiguration('control_output')
    log_level = LaunchConfiguration('log_level')

    three_axis_can_sender = Node(
        package='can_motor_controller',
        executable='three_axis_can_sender',
        name='three_axis_can_sender',
        output=control_output,
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'can_id': 0x51,
            'can_reconnect_interval': LaunchConfiguration('can_reconnect_interval'),
            'warn_interval_sec': LaunchConfiguration('can_warn_interval_sec'),
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('start_can_sender')),
    )

    dynamic_target = Node(
        package='can_motor_controller',
        executable='dynamic_target',
        name='dynamic_target',
        output=control_output,
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('start_dynamic_target')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_output',
            default_value='screen',
            description='Output mode for CAN nodes: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='ROS log level for CAN nodes'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        DeclareLaunchArgument(
            'start_can_sender',
            default_value='true',
            description='Whether to start the three-axis CAN sender node'
        ),
        DeclareLaunchArgument(
            'can_reconnect_interval',
            default_value='5.0',
            description='CAN reconnect interval in seconds'
        ),
        DeclareLaunchArgument(
            'can_warn_interval_sec',
            default_value='5.0',
            description='CAN warning throttle interval in seconds'
        ),
        DeclareLaunchArgument(
            'start_dynamic_target',
            default_value='true',
            description='Whether to start the dynamic target publisher'
        ),
        three_axis_can_sender,
        dynamic_target,
    ])
