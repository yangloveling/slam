#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # 是否启动 dynamic_target 节点
    dynamic_target_arg = DeclareLaunchArgument(
        'dynamic_target',
        default_value='true',
        description='Whether to start the dynamic target publisher'
    )

    setup_can_arg = DeclareLaunchArgument(
        'setup_can',
        default_value='true',
        description='Whether to bring up the CAN interface before starting CAN nodes'
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )

    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='1000000',
        description='CAN interface bitrate'
    )

    start_can_sender_arg = DeclareLaunchArgument(
        'start_can_sender',
        default_value='true',
        description='Whether to start the three-axis CAN sender node'
    )

    can_reconnect_interval_arg = DeclareLaunchArgument(
        'can_reconnect_interval',
        default_value='5.0',
        description='CAN reconnect interval in seconds'
    )

    can_warn_interval_sec_arg = DeclareLaunchArgument(
        'can_warn_interval_sec',
        default_value='5.0',
        description='CAN warning throttle interval in seconds'
    )

    sensor_output_arg = DeclareLaunchArgument(
        'sensor_output',
        default_value='log',
        description='Output mode for sensor/static TF nodes: screen, log, or both'
    )

    control_output_arg = DeclareLaunchArgument(
        'control_output',
        default_value='screen',
        description='Output mode for control nodes: screen, log, or both'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='WARN',
        description='ROS log level for bringup nodes'
    )

    can_setup = ExecuteProcess(
        cmd=[
            'sudo', '-n', 'ip', 'link', 'set',
            LaunchConfiguration('can_interface'),
            'up', 'type', 'can', 'bitrate',
            LaunchConfiguration('can_bitrate')
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_can')),
    )

    three_axis_can_sender = Node(
        package='can_motor_controller',
        executable='three_axis_can_sender',
        name='three_axis_can_sender',
        output=LaunchConfiguration('control_output'),
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'can_id': 0x51,
            'can_reconnect_interval': LaunchConfiguration('can_reconnect_interval'),
            'warn_interval_sec': LaunchConfiguration('can_warn_interval_sec'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('start_can_sender')),
    )

    # 动态目标节点
    # 该节点发布 /target_pos
    dynamic_target = Node(
        package='can_motor_controller',
        executable='dynamic_target',
        name='dynamic_target',
        output=LaunchConfiguration('control_output'),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('dynamic_target')),
    )

    # base_link -> imu_frame 静态 TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_static_tf',
        output=LaunchConfiguration('sensor_output'),
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_frame'
        ]
    )

    return LaunchDescription([

        dynamic_target_arg,
        setup_can_arg,
        can_interface_arg,
        can_bitrate_arg,
        start_can_sender_arg,
        can_reconnect_interval_arg,
        can_warn_interval_sec_arg,
        sensor_output_arg,
        control_output_arg,
        log_level_arg,

        can_setup,

        TimerAction(
            period=1.0,
            actions=[
                three_axis_can_sender,
                dynamic_target,
                static_tf,
            ]
        ),
    ])
