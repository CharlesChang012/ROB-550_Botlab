# ROB 550 Botlab-w25 @ UMich
## Overview
This repository presents the outcomes of the ROB-550 BotLab course at University of Michigan. University of Michigan’s mobile robot - Mbot Classic\cite{MbotClassic}, is implemented along with controls, SLAM (Simultaneous Localization and Mapping) system, and camera vision to interact with its surroundings. For basic control system, wheel odometry, a Bosch BHI160B IMU (Inertial Measurement Unit) and PID (Proportional - Integral - Derivative) control are applied. Through fine-tuning the control values, motion errors are corrected and thus it ensures desired motion along its path. SLAM system is implemented by integrating occupancy grid mapping for representation of the world and the widely-used particle filter estimation method with RPLiDAR sensor for localization. Arducam 5MP OV5647 camera module is equipped for real-time image capturing. The performance and accuracy of the implementation is analyzed and tested through a series of experiments. 


**Table of content**
- [Results](#results)
- [Code structure](#code-structure)
- [How to start](#how-to-start)
- [Reference](#reference)

## Results
### PID controll for navigation through maze with waypoints
- **Smart Motion Controller**  
  <div style="display: flex; justify-content: space-between;">
    <img src="media/cp1_smart_low_speed.jpg" alt="Maze smart motion controller low speed" style="width: 45%;" />
    <img src="media/cp1_smart_high_speed.jpg" alt="Maze smart motion controller high speed" style="width: 45%;" />
  </div>

- **Pure Pursuit Motion Controller**  
  <div style="display: flex; justify-content: space-between;">
    <img src="media/CP1_pure_pursuit_low_speed.jpg" alt="Maze pure pursuit motion controller low speed" style="width: 45%;" />
    <img src="media/CP1_pure_pursuit_high_speed.jpg" alt="Maze pure pursuit motion controller high speed" style="width: 45%;" />
  </div>

### PID controll for navigation in square with waypoints
- **Smart Motion Controller (left)** and **Pure Pursuit Motion Controller(right)**  
  <div style="display: flex; justify-content: space-between;">
    <img src="media/drive_square_smart_slow.png" alt="Square smart motion controller low speed" style="width: 45%;" />
    <img src="media/drive_square_purePursuit_slow.png" alt="Square pure pursuit motion controller low speed" style="width: 45%;" />
  </div>

### Particle Filter Performances
- **Particle Filter Update Time vs Particle Number**
  <div style="display: flex; justify-content: space-between;">
    <img src="media/PF_update_time.png" alt="Particle Filter Update Time vs Particle Number" style="width: 45%;" />
  </div>

- **SLAM RMSE and STD**
  <div style="display: flex; justify-content: space-between;">
    <img src="media/SLAM_pose_stat.jpg" alt="SLAM RMSE and STD" style="width: 45%;" />
  </div>

- **Robot SLAM Pose and Particles**
  <div style="display: flex; justify-content: space-between;">
    <img src="media/Robot_SLAM_pose_particle.png" alt="Robot SLAM pose particle" style="width: 45%;" />
  </div>

## Code structure

### Relevant
- [mbot_autonomy](mbot_autonomy)
- [mbot_bridge](mbot_bridge)
- [mbot_firmware](mbot_firmware)
- [mbot_gui](mbot_gui)
- [mbot_lcm_base](mbot_lcm_base)
- [mbot_web_app-v2.0.0](mbot_web_app-v2.0.0)
- [rplidar_lcm_driver](rplidar_lcm_driver)
- [mbot_plot](mbot_plot)
- [mbot_vision](mbot_vision)


### Irrelevant
No need to touch these files.
- [media](media) - where I store media that used for README instructions

## How to start?
Not yet ready...

## Reference
ROB 550 Web Page : https://rob550-docs.github.io/docs/botlab/getting-started.html
