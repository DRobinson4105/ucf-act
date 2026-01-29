#!/usr/bin/env bash
set -euo pipefail

echo "Installing ROS 2 and necessary packages"

sudo apt-get update && sudo apt-get upgrade -y

sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

PKGS=(
    cmake
    build-essential
    python3-colcon-common-extensions
    ros-humble-ros-base
    ros-humble-usb-cam
    ros-humble-robot-localization
    ros-humble-ublox-dgnss
    ros-humble-tf2-ros
    ros-humble-nav-msgs
)

sudo apt-get install -y "${PKGS[@]}"

echo "Installed"
echo "Building ROS 2 workspace"

[[ "$(pwd)" == */ros2 ]] || { echo "Error: must be in ros2 workspace (ucf-act/packages/ros2)"; exit 1; }
export ACT_ROS_WS="$(pwd)"

source /opt/ros/humble/setup.bash

git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
cd src/livox_ros_driver2
./build.sh humble
cd ../..

colcon build
source install/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ${ACT_ROS_WS}/install/setup.bash" >> ~/.bashrc

echo "Built"
