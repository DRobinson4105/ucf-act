import os.path as osp

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = osp.join(get_package_share_directory("perception"), "config", "seg.yaml")
    model_path = osp.join(get_package_share_directory("perception"), "models", "seg.engine")
    
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
            executable="seg_node",
            name="seg_node",
            namespace=camera,
            parameters=[
              params,
              { "trt_model_path": model_path }
            ],
            output="screen"
        ),
    ])
