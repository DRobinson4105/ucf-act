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
        #include("bringup", "serial_bridge.launch.py"),
        include("bringup", "cameras.launch.py"),
        include("livox_ros_driver2", "msg_MID360_launch.py", launch_dir="launch_ROS2"),
        include("bringup", "gps.launch.py"),
        include("fast_lio", "mapping.launch.py", launch_arguments={"rviz": "false"}),
        include("bringup", "localization.launch.py"),
        #include("perception", "seg_all.launch.py"),
        include("perception", "lidar_filter.launch.py"),
        include("bringup", "navigation.launch.py"),

        Node(
            package="navigation",
            executable="global_path_manager",
            name="global_path_manager",
            output="screen",
            parameters=[{
                "map_frame": "odom",
                "input_topic": "/ui/global_route_wgs84_json",
                "output_topic_clean": "/global_path",
                "output_topic_raw": "/global_path_raw",
                "auto_datum_from_fix": True,
                "fix_topic": "/fix",
                # "datum_lat": 28.6017759, # Set auto_datum_from_fix -> False to use hardcoded datum
                # "datum_lon": -81.2005371,
                "datum_alt": 0.0,
                "datum_covariance_threshold_m2": 25.0,
                "datum_wait_timeout_s": 5.0,
                "current_fix_max_age_s": 3.0,
                "v_max_mps": 3.58,
                "a_lat_max_mps2": 1.0
            }],
        ),
        # Node(
        #     package="navigation",
        #     executable="clicked_point_to_path",
        #     name="clicked_point_to_path",
        #     output="screen",
        #     parameters=[{
        #         "global_frame": "odom",
        #         "base_frame": "base_link",
        #         "clicked_topic": "/clicked_point",
        #         "path_topic": "/global_path",
        #         "spacing_m": 0.75
        #     }],
        # ),
        Node(
            package="navigation",
            executable="cmd_vel_adapter",
            name="cmd_vel_adapter",
            output="screen",
            parameters=[{
                "cmd_in_topic": "/cmd_vel_nav",
                "odom_topic": "/odometry/local",
                "can_topic": "/act/drive_cmd",
                "speed_limit_topic": "/speed_limit",
                "publish_hz": 25.0,
                "v_max": 3.58,
                "w_max": 1.28,
                "a_max": 1.0,
                "d_max": 1.5,
                "w_acc_max": 2.0,
                "speed_limit_timeout_s": 1.0,
                "min_turning_r": 2.8
            }],
        ),
    ])
