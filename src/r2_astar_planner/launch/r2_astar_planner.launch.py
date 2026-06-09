from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_output = LaunchConfiguration('control_output')
    log_level = LaunchConfiguration('log_level')

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
        Node(
            package='r2_astar_planner',
            executable='r2_astar_planner_node',
            name='r2_astar_planner_node',
            output=control_output,
            parameters=[{
                'start_block': 0,
                'start_yaw': 0,
                'allow_pass_r1': True,
                'allow_pass_fake': False,
                'wait_all_blocks_before_plan': False,
                'auto_plan_when_update': True,
                'enable_initial_observation_turn': True,
                'plan_requires_initial_observation': True,
                'initial_observation_dyaw_deg': 45.0,
                'initial_observation_wait_after_accept_s': 1.5,
                'initial_observation_retry_period_ms': 500,
                'relative_goal_service': '/set_relative_goal',
                'odom_topic': '/odometry/filtered',
                'lost_confirm_time': 0.5,
                'r1_monitor_period_ms': 50,
                'replan_after_r1_cleared': True,
                'use_current_block_as_replan_start': True,
                'target_count': 2,
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
    ])
