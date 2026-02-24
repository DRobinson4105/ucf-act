import os.path as osp

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    symlinks = {
        "front_left": "platform-3610000.usb-usb-0:4.1.2:1.0-video-index0",
        "side_FL": "platform-3610000.usb-usb-0:4.1.3:1.0-video-index0",
        "side_BL": "platform-141a0000.pcie-pci-0005:01:00.1-usb-0:1:1.0-video-index0",
        "side_FR": "platform-141a0000.pcie-pci-0005:01:00.3-usb-0:1:1.0-video-index0",
        "side_BR": "platform-141a0000.pcie-pci-0005:01:00.5-usb-0:1:1.0-video-index0",
        "front_right": "platform-141a0000.pcie-pci-0005:01:00.7-usb-0:1:1.0-video-index0",
    }

    devices = {k: osp.realpath(f"/dev/v4l/by-path/{symlink}") for k, symlink in symlinks.items()}

    params = osp.join(get_package_share_directory("bringup"), "config", "camera.yaml")

    nodes = [
      Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace=camera,
        name="usb_cam",
        parameters=[params, {
            "video_device": device,
            "frame_id": camera,
        }],
        output="screen",
      ) for camera, device in devices.items()
    ]

    return LaunchDescription(nodes)

