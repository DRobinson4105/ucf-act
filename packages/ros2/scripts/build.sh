#!/usr/bin/env bash
set -euo pipefail

cd ${ACT_ROS_WS}

rosdep install --from-paths src --ignore-src -y

if [ -n "${1:-}" ]; then
  colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble --packages-select $1
else
  rm -rf install log build
  colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
fi

set +u
source install/setup.bash
set -u
