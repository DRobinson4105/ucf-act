import os.path as osp

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("camera")

    symlinks = [
        "platform-3610000.usb-usb-0:4.3:1.0-video-index0",
        "platform-3610000.usb-usb-0:4.4:1.0-video-index0",
        "platform-141a0000.pcie-pci-0005:01:00.1-usb-0:1:1.0-video-index0",
        "platform-141a0000.pcie-pci-0005:01:00.3-usb-0:1:1.0-video-index0",
        "platform-141a0000.pcie-pci-0005:01:00.5-usb-0:1:1.0-video-index0",
        "platform-141a0000.pcie-pci-0005:01:00.7-usb-0:1:1.0-video-index0",
    ]

    devices = [osp.realpath(f"/dev/v4l/by-path/{symlink}") for symlink in symlinks]

    cameras = [
        "front_left",
        "front_right",
        "side_FL",
        "side_FR",
        "side_BL",
        "side_BR",
    ]

    params = osp.join(pkg_share, "config", "params.yaml")

    nodes = [Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace=f"{camera}_cam",
        name="usb_cam",
        parameters=[params, {
            "video_device": device,
            "frame_id": f"{camera}_cam",
        }],
        output="screen",
    ) for camera, device in zip(cameras, devices)]

    return LaunchDescription(nodes)

