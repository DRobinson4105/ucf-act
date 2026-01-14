#!/usr/bin/env bash
set -euo pipefail

echo "Installing ROS 2 and necessary packages"

sudo apt-get update && sudo apt-get upgrade -y

sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get update

PKGS=(
    ros-humble-desktop
    python3-colcon-common-extensions
    ros-humble-usb-cam
)

sudo apt-get install -y "${PKGS[@]}"

echo "Installed"
echo "Building ROS 2 workspace"

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
printf '\nsource /opt/ros/humble/setup.bash\nsource ~/ucf-act/packages/ros2/install/setup.bash\n' >> ~/.bashrc

echo "Built"
