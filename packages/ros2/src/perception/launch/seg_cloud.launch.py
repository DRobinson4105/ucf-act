import os.path as osp

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = osp.join(get_package_share_directory("perception"), "config", "seg_cloud.yaml")
    
    camera = LaunchConfiguration("camera")
    cameras = [
        "front_left",
        "front_right",
        "side_FL",
        "side_FR",
        "side_BL",
        "side_BR",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("camera", choices=cameras),
        Node(
            package="perception",
            executable="seg_cloud_node",
            name="seg_cloud_node",
            namespace=camera,
            parameters=[
              params,
              { "camera_frame": camera }
            ],
            output="screen"
        ),
    ])
