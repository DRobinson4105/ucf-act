import os.path as osp

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = osp.join(get_package_share_directory("bringup"), "config", "localization.yaml")

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            parameters=[params],
            remappings=[
                ("/imu", "/livox/imu"),
                ("/gps/fix", "/fix"),
                ("/odometry/filtered", "/Odometry"),
                ("/odometry/gps", "/odometry/gps"),
            ],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global_node",
            parameters=[params],
            remappings=[
                ("/odometry/filtered", "/odometry/filtered"),
            ],
            output="screen",
        )
    ])
