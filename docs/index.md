# Home

## AeroFusion: Autonomous BLIMP Navigation and Sensor Integration Platform

## 1. Repository Structure Review

Before diving into the project details, we reviewed the cloned [template](https://embedded-systems-design.github.io/fork-report-website/)  to understand how the folder structure translates to the website. We removed all markdown pages except for **index.md** to prepare the report. This clean setup ensures that our final website only displays the main home page, keeping the structure clear and uncluttered.

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

Our project aims to develop an integrated, sensor-driven framework that enables a Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) UAV to operate autonomously in dynamic and uncertain environments.

The central research question is: **"How effectively can sensor data from various sensors (such as Time-of-Flight Sensor, IMU, Barometer, Camera) be fused together to enhance trajectory planning and autonomous navigation capabilities of a hybrid robotic blimp system in dynamic environments?"**
  
![High-Level System Concept](./figures/blimp_model.JPG)  
*Figure 1: CAD Rendering of Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) UAV*

### 3.2 Sensor Integration

**How sensor data will be utilized in code:**

Sensor integration follows a systematic, multi-layered approach to harness data effectively throughout development. In our code, we'll publish sensor data on dedicated ROS 2 topics, where each sensor (IMU, barometer, GPS, Raspberry Pi camera, sonar, and LiDAR) continuously streams measurements. This modular architecture allows sensor outputs to be processed both independently and as part of a sensor fusion algorithm, enhancing real-time decision-making and control.

**Sensors to be integrated:**

- IMU: Provides high-frequency data on orientation, angular velocity, and acceleration for precise attitude estimation and stabilization
- Barometer: Measures atmospheric pressure changes to determine altitude variations for vertical stability
- GPS: Delivers geospatial coordinates and velocity information for outdoor localization and navigation
- Raspberry Pi Camera: Captures visual data for localization and goal detection
- Sonar: Emits sound waves to detect objects and measure distances for obstacle detection
- ToF Sensor (LiDAR): Generates 3D maps of surroundings for improved obstacle detection and mapping

**How sensors will be used during testing:**

During testing, individual sensor outputs are validated using ROS 2 tools like rqt_plot and ros2 topic echo, ensuring that each sensor is correctly calibrated and operating within expected parameters. This stage not only verifies the performance of the sensors in isolation but also provides critical feedback for refining sensor fusion strategies. The testing phase includes both controlled indoor experiments and field trials, allowing the team to observe how sensor data influences the system’s stability, localization accuracy, and obstacle detection in varying environments.

**How sensors will be used in final demonstration:**

In the final demonstration, the real-time integration of sensor data is pivotal. The IMU contributes to precise orientation control, while the barometer maintains altitude, and the GPS offers robust localization. Simultaneously, the Raspberry Pi camera supports visual goal detection and aids in dynamic decision-making, and the sonar along with LiDAR enhance the system's ability to detect and avoid obstacles. This cohesive sensor data integration not only drives the autonomous control loops—enabling adaptive trajectory planning and responsive mode switching between manual and autonomous controls—but also showcases the system’s comprehensive ability to operate reliably in real-world, dynamic scenarios.


### 3.3 Interaction and Interface Development

**How we plan to influence robot behavior:**

The robot will use a dual-mode control strategy:

- **Manual Mode**: Users can directly adjust throttle, direction, and altitude via joystick for precise control
- **Autonomous Mode**: The robot employs trajectory planning algorithms based on goal positions detected by the onboard camera
- A dedicated joystick button will facilitate seamless switching between manual and autonomous operations


**Interfaces for viewing, interaction, and data storage:**

We are developing a robust GUI based on ROS that will:
- Provide real-time visualization of all sensor data (IMU, barometer, GPS, camera, sonar, LiDAR)
- Display current control status (manual or autonomous)
- Present a live camera feed with overlay indicating goal detection
- Log performance data for subsequent analysis

### 3.4 Control and Autonomy

The proposed system integrates sensor feedback across both low-level control and high-level decision-making processes. In manual operation, user inputs via a joystick directly set parameters such as thrust, direction, and altitude. Conversely, during autonomous navigation, the Raspberry Pi Camera identifies goal positions that inform trajectory planning algorithms. In tandem, continuous feedback from the IMU and barometer maintains altitude stability by providing real-time orientation and pressure data. This sensor data is then fed into a dynamic feedback loop, where PID controllers perform immediate trajectory corrections, with plans to evolve towards model predictive control (MPC) for enhanced adaptive performance. Overall, by seamlessly connecting sensor inputs to both the controller and strategic decision layers, the system ensures robust and responsive behavior under varying operational conditions.

### 3.5 Prepartion

To achieve complete autonomy of the BLIMP, a comprehensive understanding of several interrelated technical areas. At a foundational level, proficiency in trajectory planning algorithms is essential for designing efficient navigation paths, while sensor fusion techniques are critical for integrating data from various sensors to achieve accurate state estimation. Additionally, expertise in ROS 2 GUI development is necessary to create an intuitive interface for real-time data visualization and system interaction.

In terms of classroom topics, it would be beneficial for us if you would cover topics such as the principles and practical applications of PID controllers and Model Predictive Control (MPC) within ROS, as these control strategies underpin our approach to ensuring stable and responsive system behavior. Further, in-depth instruction on ROS2 GUI implementation will equip the team with the skills needed to develop a robust user interface.

Understanding UAV control dynamics and the use of remote SSH to access the Raspberry Pi desktop environment are also important to effectively manage the system’s operation and debugging processes. These areas of focus will collectively provide the technical foundation required to successfully integrate sensor data and advanced control algorithms into a cohesive autonomous system.


### 3.6 Final Demonstration

To demonstrate the work in class, the blimp will autonomously maintain stable altitude using GPS, IMU, and Barometer data, while navigating towards a goal as detected by the camera. It will also incorporate feedback from LiDAR and sonar for obstacle detection and avoidance in real-time. In terms of resources, a large indoor space (such as TECH 189) or an outdoor test site will be required for ample maneuvering space. Equipment needed includes a laptop with ROS 2 for monitoring, remote SSH for system access, a joystick for manual control, and the blimp itself, fully set up for flight. As environmental conditions vary, the robot will adapt its altitude control to account for wind changes and utilize real-time feedback from LiDAR and sonar for obstacle avoidance. This ensures the robot remains stable and responsive, even in a changing environment. The testing and evaluation plan will involve a multi-stage approach: 
1. **Unit Testing**: Ensuring each sensor produces accurate data on its own. 
2. **System Testing**: Validating how well the sensors work together and ensuring proper data integration. 
3. **Functional Testing**: Verifying that the blimp performs as expected during navigation and control, checking for stability and accurate goal localization


### 3.7 Impact of the Work

This project will significantly enhance our practical skills by providing hands-on experience with ROS 2—a cornerstone for modern robotics applications. It offers an invaluable opportunity to delve into autonomous navigation and trajectory planning while mastering the complexities of real-time sensor fusion for accurate localization. Beyond technical skills, the work promises to contribute to course development by serving as a replicable template for future autonomous blimp initiatives and by offering a detailed case study on sensor-based control systems in unconventional UAVs. By addressing these advanced topics, the project will drive us to explore new material, from adaptive control strategies to the integration of diverse sensor data, thereby broadening our understanding of both theoretical and practical aspects of robotics.

### 3.8 Advising

## 4. Weekly Milestones (Weeks 7-16)