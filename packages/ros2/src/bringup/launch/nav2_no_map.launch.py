import os.path as osp

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = osp.join(get_package_share_directory("bringup"), "config", "nav2_no_map.yaml")

    return LaunchDescription([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[params],
            remappings=[
                ("cmd_vel", "/cmd_vel_nav"),
            ],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="navigation",
            executable="route_navigator",
            name="route_navigator",
            parameters=[{
                "global_path_topic": "/global_path",
                "planner_action": "/compute_path_to_pose",
                "planner_id": "GridBased",
                "follow_action": "/follow_path",
                "controller_id": "FollowPath",
                "goal_checker_id": "general_goal_checker",
                "progress_checker_id": "progress_checker",
                "map_frame": "odom",
                "base_frame": "base_link",
                "odom_topic": "/odometry/local",
                "local_costmap_topic": "/local_costmap/costmap_raw",
                "speed_limit_topic": "/speed_limit",
                "max_start_goal_distance_m": 35.0,
                "goal_ahead_m": 25.0,
                "replan_period_s": 0.5,
                "stuck_time_s": 10.0,
                "stuck_dist_m": 0.25,
                "speed_limit_publish_hz": 10.0,
                "speed_limit_max_mps": 3.58,
                "speed_limit_crawl_mps": 0.4,
                "speed_limit_fallback_mps": 0.75,
                "speed_limit_comfort_decel_mps2": 1.0,
                "speed_limit_reaction_time_s": 1.0,
                "speed_limit_buffer_m": 3.0,
                "speed_limit_cost_threshold": 100,
                "speed_limit_cap_rise_mps2": 1.0,
                "env_sample_step_m": 0.25,
                "path_speed_v_min_mps": 0.2,
                "path_speed_a_lat_max_mps2": 1.0,
                "path_speed_k_deadband": 0.02,
                "path_speed_preview_min_m": 12.0,
                "path_speed_preview_max_m": 30.0,
                "go_to_start_speed_cap_mps": 2.0,
                "start_approach_window_m": 8.0,
                "start_approach_speed_cap_mps": 1.0,
                "start_final_window_m": 3.0,
                "start_final_speed_cap_mps": 0.6,
                "goal_approach_window_m": 10.0,
                "goal_approach_speed_cap_mps": 1.2,
                "goal_final_window_m": 4.0,
                "goal_final_speed_cap_mps": 0.6
            }],
            output="screen",
        ),
    ])
