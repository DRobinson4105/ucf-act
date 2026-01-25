# ROS 2 Workspace

## Setup

1. From the ros2 workspace root, run `bash install.sh` to install necessary packages and build the workspace.
2. Register an account with [FPRN](https://www.myfloridagps.com/sbc/Account/Register) and subscribe to all GNSS Real-Time Corrections for RTK corrections.

## Build

After making changes, rebuild with:
```bash
colcon build
source install/setup.bash
```

## Run

```bash
# All nodes
ros2 launch bringup act.launch.py

# Camera driver
ros2 launch camera cam.launch.py device:=/dev/videoX camera:=front_left
# camera:=front_left|front_right|side_FL|side_FR|side_BL|side_BR

# Lidar driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# GPS driver
ros2 launch ublox_dgnss ntrip_client.launch.py use_https:=false host:=ntrip.myfloridagps.com port:=25000 mountpoint:=ORL1 username:=${FPRN_USER} password:=${FPRN_PASS}
ros2 launch ublox_dgnss ublox_rover_hpposllh_navsatfix.launch.py