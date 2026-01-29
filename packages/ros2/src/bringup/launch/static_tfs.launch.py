import os.path as osp
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tfs_path = osp.join(get_package_share_directory("bringup"), "config", "static_tfs.yaml")
    with open(tfs_path, "r") as fp:
        tfs = yaml.safe_load(fp)

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{tf['frame-id']}_{tf['child-frame-id']}",
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
