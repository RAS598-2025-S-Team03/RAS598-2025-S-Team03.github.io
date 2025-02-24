<<<<<<< HEAD
# AeroFusion 
Autonomous Control for Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) Navigation and Sensor Integration Platform
=======
# AeroFusion: Autonomous BLIMP Navigation and Sensor Integration Platform
>>>>>>> 8a9f12ae15503b0082ecc03ad6aeab54c841c96d

## 1. Repository Structure Review

## Introduction

Before diving into the project details, we reviewed the cloned template to understand how the folder structure translates to the website. We removed all markdown pages except for **index.md** to prepare the report. This clean setup ensures that our final website only displays the main home page, keeping the structure clear and uncluttered.

---

## 2. Title / Home Page

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

---

## 3. Project Plan

### 3.1 High-Level Concept and Research Question

Our project aims to to develop an integrated, sensor-driven framework that enables an Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) UAV to operate autonomously in dynamic and uncertain environments. The central research question is: **"How effectively can sensor data from various sensors (such as Time-of-Flight Sensor, IMU, Barometer, Camera) can be fused together to trajectory planning, and autonomous navigation capabilities of a hybrid robotic blimp system in dynamic environments?"** 
The experiment involves using the UR5 to position the quadruped at a designated start point, triggering the quadruped to run in a specified direction, and collecting sensor data during its motion. We then broadcast this data to a server for analysis, comparing it with simulated data. The refined simulation results are subsequently applied back to the robot to evaluate improvements in performance.
  
![High-Level System Concept](./figures/blimp_model.JPG)  
*Figure 1: CAD Rendering of Biologically-inspired, Lighter-than-air, Instructional, Mechatronics Program (BLIMP) UAV*

### 3.2 Interaction and Interface Development

### 3.3 Control and Autonomy

### 3.4 Sensor Integration

Sensor integration is critical to our experimental setup. We plan to collect a range of sensor data (e.g., IMU, Barometer, Sonar, GPS) during operation of the BLIMP.

- IMU: An inertial measurement unit that provides high-frequency data on orientation, angular velocity, and acceleration to ensure precise attitude estimation and dynamic stabilization.
- Barometer: A pressure sensor that measures atmospheric pressure changes to accurately determine altitude variations, which is critical for maintaining vertical stability.
- GPS: A global positioning module that delivers reliable geospatial coordinates and velocity information, essential for outdoor localization and navigation.
- Raspberry Pi Camera: A compact imaging device that captures high-resolution visual data for localization and goal detection, thereby enhancing situational awareness.
- Sonar: An ultrasonic sensor that emits sound waves to detect objects and measure distances in real-time, facilitating effective obstacle detection and collision avoidance.
- ToF Sensor (LiDAR): A laser-based sensor that generates detailed three-dimensional maps of the surrounding environment, significantly improving obstacle detection and spatial mapping.

### 3.5 Prepartion

### 3.6 Final Demonstration

### 3.7 Impact of the Work

### 3.8 Advising

## 4. Weekly Milestones (Weeks 7-16)