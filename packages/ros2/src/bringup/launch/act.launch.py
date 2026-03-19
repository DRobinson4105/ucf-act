import os.path as osp

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def include(pkg: str, launch_file: str, launch_dir: str = "launch", launch_arguments: dict | None = None):
    launch_arguments = launch_arguments or {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory(pkg), launch_dir, launch_file)
        ),
        launch_arguments=launch_arguments.items()
    )


def generate_launch_description():
    return LaunchDescription([
        include("bringup", "static_tfs.launch.py"),
        #include("ros2_socketcan", "socket_can_sender_node"),
        include("bringup", "cameras.launch.py"),
        include("livox_ros_driver2", "msg_MID360_launch.py", launch_dir="launch_ROS2"),
        include("bringup", "gps.launch.py"),
        include("fast_lio", "mapping.launch.py"),
        include("bringup", "localization.launch.py"),
        include("perception", "seg_all.launch.py"),
        include("perception", "lidar_filter.launch.py"),

        include("bringup", "nav2_no_map.launch.py"),


        Node(
            package="navigation",
            executable="clicked_point_path_publisher",
            name="clicked_point_path_publisher",
            output="screen",
            parameters=[{
                "global_frame": "odom",
                "base_frame": "base_link",
                "clicked_topic": "/clicked_point",
                "path_topic": "/global_path",
                "spacing_m": 0.75
            }],
        ),
        Node(
            package="navigation",
            executable="cmd_vel_adapter",
            name="cmd_vel_adapter",
            output="screen",
            parameters=[{
                "cmd_in_topic": "/cmd_vel_nav",
                "odom_topic": "/odometry/local",
                "out_topic": "/act/drive_cmd",
                "speed_limit_topic": "/speed_limit",
                "publish_hz": 25.0,
                "v_max": 3.58,
                "w_max": 1.28,
                "a_max": 1.0,
                "d_max": 1.5,
                "w_acc_max": 2.0,
                "speed_limit_timeout_s": 0.5,
                "min_turning_r": 2.8
            }],
        ),
    ])
