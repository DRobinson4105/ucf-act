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
            package="route_navigator",
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
                "map_frame": "map",
                "base_frame": "base_link",
                "goal_ahead_m": 25.0,
                "replan_period_s": 0.5,
                "stuck_time_s": 3.0,
                "stuck_dist_m": 0.25
            }],
            output="screen",
        ),
    ])
