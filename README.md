# ROB 550 Botlab-w25 @ UMich
## Overview
Not yet ready...


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
<div style="display: flex; justify-content: space-between;">
  <figure style="width: 45%; text-align: center;">
    <img src="media/drive_square_smart_slow.png" alt="Square smart motion controller low speed" style="width: 100%;" />
    <figcaption>Smart Motion Controller</figcaption>
  </figure>
  <figure style="width: 45%; text-align: center;">
    <img src="media/drive_square_purePursuit_slow.png" alt="Square pure pursuit motion controller low speed" style="width: 100%;" />
    <figcaption>Pure Pursuit Motion Controller</figcaption>
  </figure>
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
