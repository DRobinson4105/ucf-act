from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path as osp


def generate_launch_description() -> LaunchDescription:
    params = osp.join(get_package_share_directory("bringup"), "config", "esp32_link.yaml")

    return LaunchDescription([
        Node(
            package="bringup",
            executable="esp32_link_node",
            name="esp32_link",
            output="screen",
            parameters=[params],
        ),
    ])
