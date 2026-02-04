import os.path as osp

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    device = LaunchConfiguration("device")
    camera = LaunchConfiguration("camera")

    cameras = [
        "front_left",
        "front_right",
        "side_FL",
        "side_FR",
        "side_BL",
        "side_BR",
    ]
    
    params = osp.join(get_package_share_directory("bringup"), "config", "camera.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("device"),
        DeclareLaunchArgument("camera", choices=cameras),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=camera + "_cam",
            name="usb_cam",
            parameters=[params, {
                "video_device": device,
                "frame_id": camera + "_cam",
            }],
            output="screen",
        )
    ])
