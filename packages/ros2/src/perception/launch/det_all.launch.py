import os.path as osp

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def include(pkg: str, launch_file: str, launch_dir: str = "launch", launch_arguments: dict | None = None):
    launch_arguments = launch_arguments or {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory(pkg), launch_dir, launch_file)
        ),
        launch_arguments=launch_arguments.items()
    )

def generate_launch_description():
  cameras = [
    "front_left",
    "front_right",
    "side_FL",
    "side_FR",
    "side_BL",
    "side_BR",
  ]

  return LaunchDescription([
    include("perception", "det.launch.py", launch_arguments={"camera": camera})
    for camera in cameras
  ])
