from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path as osp


def generate_launch_description() -> LaunchDescription:
    params = osp.join(get_package_share_directory("bringup"), "config", "serial_bridge.yaml")

    return LaunchDescription([
        Node(
            package="bringup",
            executable="serial_bridge_node",
            name="serial_bridge",
            output="screen",
            parameters=[params],
        ),
    ])