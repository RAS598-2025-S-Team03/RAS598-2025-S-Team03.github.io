---
title: ROS Integration
---

## Code Repository

https://github.com/RAS598-2025-S-Team03/BLIMP-Packages

## Overview of Code:


### Detect_cpp:
- **Location:**
   - wvu_blimps_ros2_src/sensors_cpp/src/detect_node.cpp
- **File Type:**
   - C++
- **Executable Name:**
   - detect_cpp
- **Dependencies:** 
   - rclcpp.hpp: ROS 2 C++ communication package
   - camera_coord.hpp: custom interface from blimp_interfaces to publish the camera coordinates
   - joy.hpp: interfaces from sensor_msgs/msg to subscribe to the /joy topic and read controller inputs.
   - opencv.hpp: Package to use OpenCV 
   - vector: Package to use vectors and vector math
- **Parameters:** 
   - None
- **Subscriptions:** 
   - /joy
- **Publications:**
   - /cam_data
- **Descriptions:**
   - This node captures an image from the camera every time it runs. It has two modes: detect balloon or detect goal. To change between these modes, the pilot needs to press the X button on the Xbox controller. In balloon detection mode, the program will use the defined range of HSV values and look for the area with the highest density of those values. The program will then draw a circle around the detected density. If the circle is bigger than the minimum radius, the center coordinates of the circle are sent to an averaging function. This function will publish the center coordinates for every number of detections we define.

   - In goal detection, the program works similarly to balloon detection. We define a set range of HSV values to focus on the colors we need. From there, we find the edges of the color detected using cv2.canny. Using those edges, the OpenCV method, HoughLinesP, finds probabilistic lines and saves their x1, y1, x2, and y2. Using these 2 points, we find the midpoints of the line. Taking the max and min X of the midpoints(meaning the 2 vertical sides of the goal) and the max and min Y(meaning the 2 horizontal sides of the goal), we find the average of those to give us the center of the detected goal.  

### Esc_driver:
- **Location:** 
   - wvu_blimps_ros2_src/controls/controls/esc_driver.py  
- **File Type:** 
   - python3  
- **Executable Name:** 
   - esc_driver  
- **Dependencies:** 
   - rclpy: ROS 2 Python communications package
   - EscInput: custom interface from blimp_interfaces/msg for publishing and reading PWM inputs.
   - Joy: interfaces from sensor_msgs/msg to subscribe to the /joy topic and read controller inputs.
   - pigpio: library used to change the PWM signals on the specified pins.
   - os: Allows the program to run a command in the terminal
   - time: used for sleep or getting the local time
   - subprocess: used to access the Bluetooth stuff
- **Parameters:** 
   - MAC: import the MAC address of the current controller.  
- **Subscriptions:** 
   - /joy  
- **Publications:** 
   - None  
- **Descriptions:**
   - This program is used to take in an ESC input and apply it to the motors using pigpio. The ESC inputs come from subscribing to the mode switcher.
###  F_to_esc:
- **Location:** 
   - wvu_blimps_ros2_src/sensors_cpp/src/force_to_ESC_input.cpp  
- **File Type:** 
   - C++  
- **Executable Name:** 
   - F_to_esc  
- **Dependencies:** 
   - rclcpp.hpp: ROS 2 C++ communication package
   - cart_coord.hpp: custom interface from blimp_interfaces/msg for publishing and reading Cartesian.
   - esc_input.hpp: custom interface from blimp_interfaces/msg for publishing and reading pwm inputs.
   - Eigen/Dense: for doing matrix math in C++
   - cmath: For more math functionality in C++
- **Parameters:** 
   - None  
- **Subscriptions:** 
   - /force  
- **Publications:** 
   - /ESC_balloon_input  
- **Descriptions:**
   - This program subscribes to the forces imported from the inv_kine nodes and uses a pseudo-inverse method to convert the forces from the blimp's body frame to motor inputs.  
### Game_controller_node:
- **Subscriptions:** 
   - /joyfeedback  
- **Publications:** 
   - /joy  
- **Description:** 
   - This node is from the Joy package install. It is used to read the inputs from the Xbox controller and publish to the /joy topic. For more information, please visit the documentation websites. https://index.ros.org/p/joy/   
### Inv_kine:
- **Location: 
   - wvu_blimps_ros2_src/sensors_cpp/src/inv_kine.cpp  
- **File Type:** 
   - C++  
- **Executable Name:**  
   - dynamic_model
- **Dependencies:** 
   - rclcpp.hpp: ros2 cpp communication package
   - imu_data.hpp: custom package
   - cart_coord.hpp
   - baro_data.hpp
   - esc_input.hpp: custom interface from blimp_interfaces/msg for publishing and reading pwm inputs.
   - Eigen/Dense: for doing matrix math in C++
   - cmath: For more math functionality in C++
   - String: library to include string data type in C++
- **Parameters:** 
   - rho_air; default = 1.225: Air density used to find the buoyancy of the balloon
   - buoyancy; default = Weight of Blimp
- **Subscriptions:** 
   - /imu_data
   - /barometer_data
   - /balloon_input
- **Publications:** 
   - /forces
- **Descriptions:**
   - Uses information from IMU, barometer, and balloon input to model the vehicle's state. IMU data includes orientation, angular velocity, and linear acceleration. Barometer data is used to approximate vertical velocity. Balloon input is used to determine vertical acceleration and yaw acceleration. This information is run through the dynamic model, and forces and moments are published.

### Joy_to_esc:
- **Location:** 
   - wvu_blimps_ros2_src/controls/controls/joy_to_esc_input.py
- **File Type:** 
   - python3
- **Executable Name:** 
   - joy_to_esc
- **Dependencies:** 
   - rclpy: ROS 2 Python communications package
   - Joy: interfaces from sensor_msgs/msg to subscribe to the /joy topic and read controller inputs.
   - EscInput: custom interface from blimp_interfaces/msg for publishing and reading PWM inputs.
   - time: used for sleep or getting the local time
- **Parameters:**
- **Subscriptions:**
- **Publications:**
- **Descriptions:**
   - This program is used to take in an ESC input and apply it to the motors using pigpio. The ESC inputs come from subscribing to the mode switcher.
### Mode_switch:
- **Location:** 
   - wvu_blimps_ros2_src/controls/controls
- **File Type:** 
   - python3
- **Executable Name:** 
   - mode_switch
- **Dependencies:**
   - ESCInput
   - CameraCoord: 
   - Joy
   - time: used for sleep or getting the local time
- **Parameters:** 
   - None
- **Subscriptions:** 
   - /ESC_Manual_input
   - /ESC_balloon_input
   - /joy
- **Publications:** 
   - /ESC_input
- **Descriptions:**

### Pi_controller:
- **Location:** 
   - wvu_blimps_ros2_src/sensors_cpp/src/test.cpp
- **File Type:** 
   - C++
- **Executable Name:** 
   - pi_controller
- **Dependencies:** 
   - rclcpp.hp: ROS 2 C++ communication package
   - cart_coord.hpp
   - camera_coord.hpp: custom interface from blimp_interfaces to publish the camera coordinates
   - baro_data.hpp
   - vector: Package to use vectors and vector math
   - cmath: For more math functionality in C++
- **Parameters:** 
   - iheight: Initial height of barometer when program starts
   - kpx: Proportional gain for yaw acceleration 
   - kix: Integral gain for yaw acceleration
   - kpyu: Proportional gain for vertical acceleration (in the upward direction)
   - kpyd: Proportional gain for vertical acceleration (in the downward direction)
   - kiy: Integral gain for vertical acceleration
   - x_goal: Goal coordinates of camera center, x_center of camera frame (320)
   - y_goal: Goal coordinates of camera center, y_center of camera frame (240)
   - kpb: Proportional gain for barometer

- **Subscriptions:** 
   - /cam_data
   - /barometer_data

- **Publications:** 
   - /balloon_input

- **Descriptions:
   - Uses Raspberry Pi camera v2.1 to approximate the center of a target via Hue Saturation Values (HSVs). Based on the x and y directional error between the target and the camera center (goal position). Vertical and yaw accelerations are tuned and sent to the inverse kinematic model.
### Read_altitude:
- **Location:** 
   - wvu_blimps_ros2_src/sensors/sensors/barometer.py 
- **File Type:** 
   - python3
- **Executable Name:** 
   - read_altitude
- **Dependencies:** 
   - time: used for sleep or getting the local time
   - board
   - adafruit_bmp3xx
   - BaroData
- **Parameters:** 
   - sea_level_pressure: pressure at sea level on a given day 
- **Subscriptions:** 
   - None 
- **Publications:** 
   - barometer_data
- **Descriptions:**
   - Approximates the current altitude of the blimp in meters based on atmospheric conditions of the day. The barometer used in the current system is an Adafruit BMP390.

### Read_imu:
- **Location:** 
   - wvu_blimps_ros2_src/sensors_cpp/src/inv_kine.cpp
- **File Type:** 
   - python3
- **Executable Name:** 
   - read_imu
- **Dependencies:** 
   - rclpy: ROS 2 Python communications package
   - time: used for sleep or getting the local time
   - board
   - busio
   - adafruit_bno055
   - numpy
   - math
   - ImuData
- **Parameters:** 
   - None
- **Subscriptions:** 
   - None
- **Publications:** 
   - /imu_data
- **Descriptions:**
   - Uses an Adafruit BNO055 absolute orientation sensor to record Euler angles, linear acceleration, and angular velocity, which is used in the dynamic model for an estimation of vehicle state.
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

