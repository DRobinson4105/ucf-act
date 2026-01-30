import os.path as osp

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = osp.join(get_package_share_directory("bringup"), "config", "costmap.yaml")

    return LaunchDescription([
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="costmap",
            name="costmap",
            parameters=[params],
            remappings=[("bond", "/bond")],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="costmap",
            name="lifecycle_manager",
            parameters=[params],
            remappings=[("bond", "/bond")],
            output="screen",
        )
    ])
