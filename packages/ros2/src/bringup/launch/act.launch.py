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
    return LaunchDescription([
        #include("bringup", "cameras.launch.py"),
        include("livox_ros_driver2", "msg_MID360_launch.py", launch_dir="launch_ROS2"),
        include("bringup", "gps.launch.py"),
        include("bringup", "costmap.launch.py"),
        include("bringup", "static_tfs.launch.py")
    ])
