# ROS 2 Workspace

## Setup

1. From the ros2 workspace root, run `bash scripts/install.sh` to install necessary packages and build the workspace.
2. Register an account with [FPRN](https://www.myfloridagps.com/sbc/Account/Register) and subscribe to all GNSS Real-Time Corrections for RTK corrections.
3. Create a .env file in the workspace root with the following keys and your values.

```
FPRN_USER=your_username
FPRN_PASS=your_password
```

## Build

After making changes, rebuild with `bash scripts/build.sh`.

## Run

To launch all nodes,
```bash
ros2 launch bringup act.launch.py
```

To launch an individual node,
```bash
# One camera driver
ros2 launch bringup camera.launch.py device:=/dev/videoX camera:=front_left
# camera:=front_left|front_right|side_FL|side_FR|side_BL|side_BR

# All camera drivers
ros2 launch bringup cameras.launch.py

# Lidar driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# GPS driver
ros2 launch bringup gps.launch.py

# Costmap
ros2 launch bringup costmap.launch.py

# Static transforms
ros2 launch bringup static_tfs.launch.py
```
