---
title: ROS Integration
---

## Code Repository

https://github.com/RAS598-2025-S-Team03/BLIMP-Packages

## Overview of Code:


### Detect_cpp
The `detect_cpp` node is located at `wvu_blimps_ros2_src/sensors_cpp/src/detect_node.cpp` and is implemented in C++. It subscribes to the `/joy` topic and publishes camera-based detections to `/cam_data`. This node captures images from the onboard camera and operates in two modes: balloon detection and goal detection, toggled using the Xbox controller. Balloon detection uses HSV color space to find the densest region matching the target color. If the region exceeds a minimum radius, the center is averaged over several frames before publishing. Goal detection uses Canny edge detection and Hough line transforms to calculate the goal’s midpoint based on detected vertical and horizontal edges.

### Esc_driver
The `esc_driver` Python node is found in `wvu_blimps_ros2_src/controls/controls/esc_driver.py`. It subscribes to the `/joy` topic and uses `pigpio` to control motor PWM signals. It translates mode-switch inputs into ESC commands and manages communication with the controller over Bluetooth.

### F_to_esc
The `F_to_esc` node, written in C++ at `wvu_blimps_ros2_src/sensors_cpp/src/force_to_ESC_input.cpp`, subscribes to force vectors and publishes motor commands. It performs a pseudo-inverse transformation to convert forces in the body frame into ESC inputs. Dependencies include Eigen for matrix math and custom interfaces for handling Cartesian and PWM data.

### Game_controller_node
This node reads Xbox controller inputs and publishes them to `/joy`. It is built on the standard ROS 2 Joy package and is responsible for providing manual input data to various control nodes.

### Inv_kine
The `inv_kine` node resides in `wvu_blimps_ros2_src/sensors_cpp/src/inv_kine.cpp`. It takes input from `/imu_data`, `/barometer_data`, and `/balloon_input` to estimate forces acting on the blimp. Using these inputs, it publishes force vectors via the `/forces` topic. It incorporates a dynamic model that uses IMU orientation, barometer-based vertical velocity, and visual feedback to determine control outputs.

### Joy_to_esc
Located in `wvu_blimps_ros2_src/controls/controls/joy_to_esc_input.py`, this Python node directly maps joystick commands to ESC inputs for manual control. It serves as a simpler alternative to autonomous control logic when operating in manual mode.

### Mode_switch
The `mode_switch` node selects between manual and autonomous control. It listens to `/ESC_Manual_input`, `/ESC_balloon_input`, and `/joy`, and publishes the final ESC command to `/ESC_input`. It is central to toggling between pilot input and autonomous operation during testing.

### Pi_controller
Implemented in C++ and found at `wvu_blimps_ros2_src/sensors_cpp/src/test.cpp`, this node acts as a PI controller. It receives camera and barometer data and outputs control commands to the inverse kinematic model. Parameters include proportional and integral gains for yaw and vertical acceleration, as well as camera frame center coordinates for error correction.

### Read_altitude
This Python node at `wvu_blimps_ros2_src/sensors/sensors/barometer.py` reads atmospheric pressure using an Adafruit BMP390 and estimates altitude. It publishes data on the `barometer_data` topic and accounts for sea level pressure calibration to improve accuracy.

### Read_imu
The `read_imu` node, written in Python, reads data from the Adafruit BNO055 IMU. It measures Euler angles, linear acceleration, and angular velocity, publishing all data to the `/imu_data` topic. This sensor information supports real-time state estimation for the blimp.

## Quick Setup Guide

There are 3 commands to run this autonomous mobile blimp.
1. Go inside the workspace’s launch file. In my case:
   - Ros2_ws → src → wvu_blimps_ros2_src→ launch
2. Now there are 3 commands to run sequentially after connecting the joystick via Bluetooth to the RPi.
   - Ros2 runs joy game_controller_node → To check if the Joystick is publishing commands.
   - Python3 Arming.py → To arm the ESCs before flight
   - Ros2 launch Updated_launch.py → To launch all the nodes together.
## 1. Prerequisites
- Several Python Libraries
- Remote SSH to rpi
- Pi GPIO

