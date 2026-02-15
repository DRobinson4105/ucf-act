import os.path as osp

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = osp.join(get_package_share_directory("bringup"), "config", "localtest.yaml")

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
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            parameters=[params],
            output="screen",
        ),
    ])
