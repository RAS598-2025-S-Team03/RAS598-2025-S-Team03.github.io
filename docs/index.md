# AeroFusion: Autonomous BLIMP Navigation and Sensor Integration Platform

## 1. Repository Structure Review

Before diving into the project details, we reviewed the cloned template to understand how the folder structure translates to the website. We removed all markdown pages except for **index.md** to prepare the report. This clean setup ensures that our final website only displays the main home page, keeping the structure clear and uncluttered.

## 2. Home Page

**Project Name:**  

_AeroFusion: Autonomous BLIMP Navigation and Sensor Integration Platform_

**Team Number:**  

Team 03

**Team Members:**  

- Nihar Masurkar
- Prajjwal Dutta
- [Sai Srinivas Tatwik Meesala](https://tatwik19.github.io/)

**Semester and Year:**  
Spring 2025

**University, Class, and Professor:**

- **University:** Arizona State University  
- **Class:** RAS 598: Experimentation and Deployment of Robotic Systems
- **Professor:** Dr. Daniel M. Aukes 


## 3. Project Plan

### 3.1 High-Level Concept and Research Question

Our project aims to to develop an integrated, sensor-driven framework that enables an Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) UAV to operate autonomously in dynamic and uncertain environments. The central research question is: **"How effectively can sensor data from various sensors (such as Time-of-Flight Sensor, IMU, Barometer, Camera) can be fused together to trajectory planning, and autonomous navigation capabilities of a hybrid robotic blimp system in dynamic environments?"** 
The experiment involves using the UR5 to position the quadruped at a designated start point, triggering the quadruped to run in a specified direction, and collecting sensor data during its motion. We then broadcast this data to a server for analysis, comparing it with simulated data. The refined simulation results are subsequently applied back to the robot to evaluate improvements in performance.
  
![High-Level System Concept](./figures/blimp_model.JPG)  
*Figure 1: CAD Rendering of Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) UAV*

### 3.2 Sensor Integration

Sensor integration in this project is approached as a systematic and multi-layered process, ensuring that the rich dataset from each sensor is effectively harnessed across all stages of development—from coding through testing to the final demonstration. In the code, sensor data is published on dedicated ROS 2 topics, where each sensor (IMU, barometer, GPS, Raspberry Pi camera, sonar, and LiDAR) continuously streams its specific measurements. This design facilitates a modular architecture where sensor outputs are processed both independently and as part of a sensor fusion algorithm, enhancing real-time decision-making and control. Various sensors would be integrated in the BLIMP structure to enable navigation autonomy such as:

- IMU: An inertial measurement unit that provides high-frequency data on orientation, angular velocity, and acceleration to ensure precise attitude estimation and dynamic stabilization.
- Barometer: A pressure sensor that measures atmospheric pressure changes to accurately determine altitude variations, which is critical for maintaining vertical stability.
- GPS: A global positioning module that delivers reliable geospatial coordinates and velocity information, essential for outdoor localization and navigation.
- Raspberry Pi Camera: A compact imaging device that captures high-resolution visual data for localization and goal detection, thereby enhancing situational awareness.
- Sonar: An ultrasonic sensor that emits sound waves to detect objects and measure distances in real-time, facilitating effective obstacle detection and collision avoidance.
- ToF Sensor (LiDAR): A laser-based sensor that generates detailed three-dimensional maps of the surrounding environment, significantly improving obstacle detection and spatial mapping.

During testing, individual sensor outputs are validated using ROS 2 tools like rqt_plot and ros2 topic echo, ensuring that each sensor is correctly calibrated and operating within expected parameters. This stage not only verifies the performance of the sensors in isolation but also provides critical feedback for refining sensor fusion strategies. The testing phase includes both controlled indoor experiments and field trials, allowing the team to observe how sensor data influences the system’s stability, localization accuracy, and obstacle detection in varying environments.

In the final demonstration, the real-time integration of sensor data is pivotal. The IMU contributes to precise orientation control, while the barometer maintains altitude, and the GPS offers robust localization. Simultaneously, the Raspberry Pi camera supports visual goal detection and aids in dynamic decision-making, and the sonar along with LiDAR enhance the system's ability to detect and avoid obstacles. This cohesive sensor data integration not only drives the autonomous control loops—enabling adaptive trajectory planning and responsive mode switching between manual and autonomous controls—but also showcases the system’s comprehensive ability to operate reliably in real-world, dynamic scenarios.


### 3.3 Interaction and Interface Development

The behavior of the robot is designed to be influenced through a dual-mode control strategy complemented by a comprehensive graphical user interface. In manual mode, users can directly adjust the robot’s throttle, direction, and altitude via a joystick, allowing for precise, hands-on control. In autonomous mode, the robot employs trajectory planning algorithms based on goal positions detected by the onboard camera, with a dedicated joystick button facilitating seamless mode switching between manual and autonomous operations.

For interfacing, we are developing a robust GUI based on ROS that serves multiple functions. It will provide real-time visualization of sensor data—including IMU, barometer, GPS, camera, sonar, and LiDAR outputs—ensuring that the operator has complete situational awareness. Additionally, the GUI will display the current control status (manual or autonomous), present a live camera feed with an overlay indicating goal detection, and log performance data for subsequent analysis. This design not only enables immediate interaction during operation but also supports detailed post-mission reviews.

### 3.4 Control and Autonomy

The proposed system integrates sensor feedback across both low-level control and high-level decision-making processes. In manual operation, user inputs via a joystick directly set parameters such as thrust, direction, and altitude. Conversely, during autonomous navigation, the Raspberry Pi Camera identifies goal positions that inform trajectory planning algorithms. In tandem, continuous feedback from the IMU and barometer maintains altitude stability by providing real-time orientation and pressure data. This sensor data is then fed into a dynamic feedback loop, where PID controllers perform immediate trajectory corrections, with plans to evolve towards model predictive control (MPC) for enhanced adaptive performance. Overall, by seamlessly connecting sensor inputs to both the controller and strategic decision layers, the system ensures robust and responsive behavior under varying operational conditions.

### 3.5 Prepartion

To achieve complete autonomy of the BLIMP, a comprehensive understanding of several interrelated technical areas. At a foundational level, proficiency in trajectory planning algorithms is essential for designing efficient navigation paths, while sensor fusion techniques are critical for integrating data from various sensors to achieve accurate state estimation. Additionally, expertise in ROS 2 GUI development is necessary to create an intuitive interface for real-time data visualization and system interaction.

In terms of classroom topics, it would be beneficial for us if you would cover topics such as the principles and practical applications of PID controllers and Model Predictive Control (MPC) within ROS, as these control strategies underpin our approach to ensuring stable and responsive system behavior. Further, in-depth instruction on ROS2 GUI implementation will equip the team with the skills needed to develop a robust user interface.

Understanding UAV control dynamics and the use of remote SSH to access the Raspberry Pi desktop environment are also important to effectively manage the system’s operation and debugging processes. These areas of focus will collectively provide the technical foundation required to successfully integrate sensor data and advanced control algorithms into a cohesive autonomous system.


### 3.6 Final Demonstration

### 3.7 Impact of the Work

### 3.8 Advising

## 4. Weekly Milestones (Weeks 7-16)