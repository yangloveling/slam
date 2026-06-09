from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    control_output = LaunchConfiguration('control_output')
    log_level = LaunchConfiguration('log_level')
    serial_log_period_ms = LaunchConfiguration('serial_log_period_ms')
    serial_log_level = LaunchConfiguration('serial_log_level')
    start_serial_reader = LaunchConfiguration('start_serial_reader')
    serial_port = LaunchConfiguration('serial_port')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_output',
            default_value='screen',
            description='Output mode for planner nodes: screen, log, or both'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='ROS log level for bringup nodes'
        ),
        DeclareLaunchArgument(
            'serial_log_period_ms',
            default_value='1000',
            description='Throttle period for serial reader decoded-frame logs in milliseconds'
        ),
        DeclareLaunchArgument(
            'serial_log_level',
            default_value='INFO',
            description='ROS log level for serial_reader_node'
        ),
        DeclareLaunchArgument(
            'start_serial_reader',
            default_value='true',
            description='Whether to start serial_reader_node'
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
        # 决策节点
        Node(
            package='r2_planner',
            executable='r2_decision_node',
            name='r2_decision_node',
            output=control_output,
            parameters=[{
                'total_blocks': 12,
                'target_count': 2,
                'wait_all_blocks_before_plan': True,
	                'auto_send_path_to_controller': True,
	                'send_path_once': True,
	                'send_changed_plan_when_path_changes': True,
	                'debug_log': True,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        # 路径规划节点（R1 避让、Dijkstra）
        Node(
            package='r2_planner',
            executable='r2_dijkstra_planner_node',
            name='r2_dijkstra_planner_node',
            output=control_output,
            parameters=[{
                'start_block': 0,
                'start_yaw': 0,
                'allow_pass_r1': True,
                'allow_pass_fake': False,
                'wait_all_blocks_before_plan': True,
                'lost_confirm_time': 0.5,          # 缩短等待时间
                'r1_monitor_period_ms': 50,
                'replan_after_r1_cleared': True,
                'use_current_block_as_replan_start': True,
                'target_count': 2,
                'final_stop_block': 16,
                'pickup_prefer_left_cost': -20.0,
                'pickup_prefer_bottom_cost': -18.0,
                'pickup_penalty_right_cost': 20.0,
                'pickup_penalty_top_cost': 18.0,
                'camera_forward_offset_x_m': 0.06,
                'camera_forward_offset_y_m': -0.32,
                'camera_backward_offset_x_m': -0.06,
                'camera_backward_offset_y_m': 0.32,
                'locked_r1_refresh_radius_m': 0.55,
                'only_track_path_relevant_r1': True,
                'id_type_topic': '/id_type',
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        # 串口读取节点（如果有）
        Node(
            package='r2_planner',
            executable='serial_reader_node',
            name='serial_reader_node',
            output=control_output,
            parameters=[{
                'log_period_ms': serial_log_period_ms,
                'serial_port': serial_port,
                'baud_rate': serial_baud_rate,
                'id_type_topic': '/detected_id_type',
                'raw_data_topic': '/serial/raw_data',
            }],
            arguments=['--ros-args', '--log-level', serial_log_level],
            condition=IfCondition(start_serial_reader),
        ),
    ])
