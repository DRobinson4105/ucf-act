#!/usr/bin/env bash
set -euo pipefail

cd ${ACT_ROS_WS}

rm -r install log build

colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
set +u
source install/setup.bash
set -u
