import os.path as osp

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = osp.join(get_package_share_directory("perception"), "config", "lidar_filter.yaml")
    
    return LaunchDescription([
        Node(
            package="perception",
            executable="lidar_filter_node",
            name="lidar_filter_node",
            parameters=[params],
            output="screen"
        ),
    ])
