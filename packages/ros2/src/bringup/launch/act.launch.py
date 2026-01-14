import os.path as osp

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def include(pkg, launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        osp.join(get_package_share_directory(pkg), "launch", launch_file)
    ))

def generate_launch_description():
    return LaunchDescription([
        include("camera", "full_cam.launch.py")
    ])

