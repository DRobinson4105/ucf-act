# Overview

Our Senior Design project, **Autonomous Campus Taxi (ACT)**, is a self-driving electric golf cart designed to provide on-demand, autonomous transportation for students and faculty across the University of Central Florida campus. In addition to showcasing autonomous systems engineering, ACT addresses real challenges faced by students such as long travel times between buildings, limited access to efficient on-campus transportation, and inadequate solutions for students and faculty with physical disabilities or impairments. By offering a convenient, inclusive, and user-friendly solution, ACT aims to improve campus accessibility and the overall student experience at UCF.

To achieve this, the golf cart will autonomously follow a predefined path at a maximum speed of 8 mph, detect on-path obstacles, and safely stop or reroute as needed. The system will be powered by a NVIDIA Jetson Orin Nano, which will handle perception, planning and communication. Actuator commands (steering, throttle, and braking) will be sent over a CAN bus to ESP32 microcontrollers responsible for low-level control and safety.

A suite of sensors, including LiDAR, RGB cameras, ultrasonic sensors, RTK GNSS, IMU, wheel speeds, and steering angle sensors will be mounted, calibrated, and time-synchronized on the vehicle. These inputs will feed into the Jetson Orin Nano for real-time decision-making. In parallel, a mobile app will allow users to request the cart, track its live location, and specify a destination on campus.

While ACT is a Senior Design project, our ambition extends beyond the classroom. We hope to spark broader interest in accessible, sustainable, and autonomous campus transportation, and pave the way for future student-led innovation at UCF.
