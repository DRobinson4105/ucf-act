**UCF Autonomous Campus Taxi** is a self-driving electric golf cart designed to provide safe, efficient, and on-demand transportation across the UCF campus for students.

The vehicle navigates via a pre-mapped network of safe, pedestrian-friendly campus paths with a path planning algorithm determining the most efficient route. Multiple RGB-D cameras, 360 degree LiDAR, and radar sensors will be installed around the cart to provide real-time 360 degree awareness to detect and avoid pedestrians, vehicles, and obstacles through various weather conditions.

Cart acceleration will be controlled by an ESP-32 module directly connected to the throttle position sensor (TPS) assembly on the cart. Braking and steering will be managed by linear actuators connected to the ESP-32. Jetson Orin and ESP-32 will communicate back and forth as Jetson Orin will handle all intense computation for cart’s real-time path planning and environment perception (based on sensors around cart).  The cart will allow the sit-in driver to override the cart's driving system at any time while being equipped with a physical and remote killswitch.

## Tech Stack

#### Software
- Python (PyTorch, TensorRT, OpenCV)
- ROS 2 (Nav2, Cartographer)
- CARLA for simulation testing
- React Native (Building UI/UX for mobile app)
- PostgreSQL or SQLite (Handling users and storing path information)

#### Hardware
- Electric golf cart (E-Z-GO TXT / Club Car DS)
- NVIDIA Jetson Orin (perception and planning)
- ESP32 (safety and actuator control)
- GNSS IMU System
- Steering and brake linear actuators
- Sensors (RGB-D cameras, 360 degree LiDAR, Radar, Ultrasonic)
