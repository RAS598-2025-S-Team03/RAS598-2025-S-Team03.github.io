---
title: Gantt Chart
---

| Week            | Hardware                | Interface            | Sensors               | Controls & Autonomy                                |
|----------------|------------------------|----------------------|----------------------|--------------------------------------------------|
| MAR 4 - MAR 10  | Finalize hardware setup | Initial GUI design  | IMU & GPS testing    | Basic manual control                             |
| MAR 11 - MAR 17 | Battery & power testing | GUI mockup          | LiDAR & sonar setup  | Joystick control tuning                         |
| MAR 18 - MAR 24 | ESC & motor tests       | GUI ROS integration | Sensor data logging  | Altitude stabilization                          |
| MAR 25 - MAR 31 | Full system wiring check| Live sensor visualization | Sensor calibration  | Basic trajectory planning                        |
| APR 1 - APR 7   | First flight test       | Data storage setup  | Sensor fusion start  | PID tuning for stability                        |
| APR 8 - APR 14  | Propeller optimization  | GUI live map view   | Camera processing    | Path following algorithm                        |
| APR 15 - APR 21 | Flight test refinements | Control panel UI    | Final sensor tuning  | Autonomous mode switch                          |
| APR 22 - APR 28 | Demo preparation        | UI testing          | Validation tests     | Final trajectory optimization and full autonomous test |
| APR 29 - MAY 5  | Final presentation prep | User guide documentation | Demo readiness      | Classroom demonstration                         |


## Gantt Chart Representation 

gantt
    title Project Timeline (March - May)
    dateFormat  YYYY-MM-DD
    
    section Hardware
    Finalize hardware setup         :done, 2024-03-04, 7d
    Battery & power testing         :done, 2024-03-11, 7d
    ESC & motor tests               :active, 2024-03-18, 7d
    Full system wiring check        :2024-03-25, 7d
    First flight test               :2024-04-01, 7d
    Propeller optimization          :2024-04-08, 7d
    Flight test refinements         :2024-04-15, 7d
    Demo preparation                :2024-04-22, 7d
    Final presentation prep         :2024-04-29, 7d
    
    section Interface
    Initial GUI design              :done, 2024-03-04, 7d
    GUI mockup                      :done, 2024-03-11, 7d
    GUI ROS integration             :active, 2024-03-18, 7d
    Live sensor visualization       :2024-03-25, 7d
    Data storage setup              :2024-04-01, 7d
    GUI live map view               :2024-04-08, 7d
    Control panel UI                :2024-04-15, 7d
    UI testing                      :2024-04-22, 7d
    User guide documentation        :2024-04-29, 7d
    
    section Sensors
    IMU & GPS testing               :done, 2024-03-04, 7d
    LiDAR & sonar setup             :done, 2024-03-11, 7d
    Sensor data logging             :active, 2024-03-18, 7d
    Sensor calibration              :2024-03-25, 7d
    Sensor fusion start             :2024-04-01, 7d
    Camera processing               :2024-04-08, 7d
    Final sensor tuning             :2024-04-15, 7d
    Validation tests                :2024-04-22, 7d
    Demo readiness                  :2024-04-29, 7d
    
    section Controls & Autonomy
    Basic manual control            :done, 2024-03-04, 7d
    Joystick control tuning         :done, 2024-03-11, 7d
    Altitude stabilization          :active, 2024-03-18, 7d
    Basic trajectory planning       :2024-03-25, 7d
    PID tuning for stability        :2024-04-01, 7d
    Path following algorithm        :2024-04-08, 7d
    Autonomous mode switch          :2024-04-15, 7d
    Final trajectory optimization   :2024-04-22, 7d
    Classroom demonstration         :2024-04-29, 7d


