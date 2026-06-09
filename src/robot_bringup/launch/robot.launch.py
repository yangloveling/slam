#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def include_launch(package, path_parts, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package),
                *path_parts,
            ])
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def arg_equals(name, value):
    return IfCondition(PythonExpression(["'", LaunchConfiguration(name), "' == '", value, "'"]))


def generate_launch_description():
    setup_can = LaunchConfiguration('setup_can')
    can_interface = LaunchConfiguration('can_interface')
    can_bitrate = LaunchConfiguration('can_bitrate')
    start_can_sender = LaunchConfiguration('start_can_sender')
    can_reconnect_interval = LaunchConfiguration('can_reconnect_interval')
    can_warn_interval_sec = LaunchConfiguration('can_warn_interval_sec')
    dynamic_target = LaunchConfiguration('dynamic_target')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    sensor_output = LaunchConfiguration('sensor_output')
    control_output = LaunchConfiguration('control_output')
    log_level = LaunchConfiguration('log_level')
    debug_print_every_ms = LaunchConfiguration('debug_print_every_ms')
    sensor_debug_log_period_sec = LaunchConfiguration('sensor_debug_log_period_sec')
    serial_log_period_ms = LaunchConfiguration('serial_log_period_ms')

    hi13s2_launch = include_launch(
        'hi13s2_imu',
        ['launch', 'imu_ekf.launch.py'],
        condition=IfCondition(LaunchConfiguration('start_hi13s2')),
        launch_arguments={
            'setup_can': setup_can,
            'can_interface': can_interface,
            'can_bitrate': can_bitrate,
            'start_can_sender': start_can_sender,
            'can_reconnect_interval': can_reconnect_interval,
            'can_warn_interval_sec': can_warn_interval_sec,
            'dynamic_target': dynamic_target,
            'sensor_output': sensor_output,
            'control_output': control_output,
            'log_level': log_level,
        },
    )

    livox_launch = include_launch(
        'livox_ros_driver2',
        ['launch_ROS2', 'msg_MID360_launch.py'],
        condition=IfCondition(LaunchConfiguration('start_livox')),
        launch_arguments={
            'sensor_output': sensor_output,
            'log_level': log_level,
            'sensor_debug_log_period_sec': sensor_debug_log_period_sec,
        },
    )

    fast_lio_mapping = TimerAction(
        period=3.0,
        actions=[
            include_launch(
                'fast_lio',
                ['launch', 'mapping.launch.py'],
                condition=arg_equals('slam', 'mapping'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'config_file': fast_lio_config_file,
                    'sensor_output': sensor_output,
                    'log_level': log_level,
                },
            )
        ],
    )

    fast_lio_relocalization = TimerAction(
        period=3.0,
        actions=[
            include_launch(
                'fastlio2_relocalization',
                ['launch', 'relocalization.launch.py'],
                condition=arg_equals('slam', 'relocalization'),
                launch_arguments={
                    'sensor_output': sensor_output,
                    'log_level': log_level,
                },
            )
        ],
    )

    robot_control = include_launch(
        'robot_control',
        ['launch', 'robot_control.launch.py'],
        condition=IfCondition(LaunchConfiguration('start_control')),
        launch_arguments={
            'control_output': control_output,
            'log_level': log_level,
        },
    )

    task_controller = include_launch(
        'lite_task_controller',
        ['launch', 'lite_task_controller.launch.py'],
        condition=IfCondition(LaunchConfiguration('start_task')),
        launch_arguments={
            'control_output': control_output,
            'log_level': log_level,
            'debug_print_every_ms': debug_print_every_ms,
        },
    )

    astar_planner = include_launch(
        'r2_astar_planner',
        ['launch', 'r2_astar_planner.launch.py'],
        condition=arg_equals('planner', 'astar'),
        launch_arguments={
            'control_output': control_output,
            'log_level': log_level,
            'debug_print_every_ms': debug_print_every_ms,
        },
    )

    dijkstra_planner = include_launch(
        'r2_planner',
        ['launch', 'r2_planner.launch.py'],
        condition=arg_equals('planner', 'dijkstra'),
        launch_arguments={
            'control_output': control_output,
            'log_level': log_level,
            'serial_log_period_ms': serial_log_period_ms,
            'start_serial_reader': LaunchConfiguration('start_serial_reader'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_hi13s2', default_value='true'),
        DeclareLaunchArgument('setup_can', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='1000000'),
        DeclareLaunchArgument(
            'start_can_sender',
            default_value='true',
            description='Whether to start the three-axis CAN sender node'
        ),
        DeclareLaunchArgument(
            'can_reconnect_interval',
            default_value='5.0',
            description='CAN reconnect interval for three_axis_can_sender in seconds'
        ),
        DeclareLaunchArgument(
            'can_warn_interval_sec',
            default_value='5.0',
            description='CAN warning throttle interval for three_axis_can_sender in seconds'
        ),
        DeclareLaunchArgument('dynamic_target', default_value='true'),

        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('slam', default_value='mapping'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fast_lio_config_file', default_value='mid360.yaml'),

        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('start_task', default_value='true'),
        DeclareLaunchArgument('planner', default_value='dijkstra'),
        DeclareLaunchArgument(
            'start_serial_reader',
            default_value='true',
            description='Whether to start serial_reader_node when planner:=dijkstra'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial device path for serial_reader_node'
        ),
        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='115200',
            description='Serial baud rate for serial_reader_node'
        ),
        DeclareLaunchArgument(
            'sensor_output',
            default_value='log',
            description='Output mode for sensor, SLAM, EKF, and static TF nodes: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'control_output',
            default_value='screen',
            description='Output mode for control, task, and planner nodes: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='ROS log level passed to bringup nodes'
        ),
        DeclareLaunchArgument(
            'debug_print_every_ms',
            default_value='1000',
            description='Throttle period for high-rate control/task debug logs in milliseconds'
        ),
        DeclareLaunchArgument(
            'sensor_debug_log_period_sec',
            default_value='5.0',
            description='Throttle period for sensor statistics logs in seconds'
        ),
        DeclareLaunchArgument(
            'serial_log_period_ms',
            default_value='1000',
            description='Throttle period for serial reader decoded-frame logs in milliseconds'
        ),

        hi13s2_launch,
        livox_launch,
        fast_lio_mapping,
        fast_lio_relocalization,
        robot_control,
        task_controller,
        astar_planner,
        dijkstra_planner,
    ])
