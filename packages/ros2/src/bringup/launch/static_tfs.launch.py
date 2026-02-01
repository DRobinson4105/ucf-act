import os.path as osp
import yaml
import re

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def filter_frame(frame: str):
    pattern = r"[^a-zA-Z0-9_]+"
    result = re.sub(pattern, '_', frame)
    return result

def generate_launch_description():
    tfs_path = osp.join(get_package_share_directory("bringup"), "config", "static_tfs.yaml")
    with open(tfs_path, "r") as fp:
        tfs = yaml.safe_load(fp)

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{filter_frame(tf['frame-id'])}_{filter_frame(tf['child-frame-id'])}",
            arguments=[
                "--x", str(tf["x"]),
                "--y", str(tf["y"]),
                "--z", str(tf["z"]),
                "--roll", str(tf["roll"]),
                "--pitch", str(tf["pitch"]),
                "--yaw", str(tf["yaw"]),
                "--frame-id", tf["frame-id"],
                "--child-frame-id", tf["child-frame-id"]
            ],
            output="screen",
        ) for tf in tfs
    ])
