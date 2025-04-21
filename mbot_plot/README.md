## Overview
This repository contains python scripts that read in log files from the mbot and generate plots.

## To Plot SLAM odometry pose and maze
```bash
$ python3 read_lcm_log.log -f [path/to/logFile] -m [maze] -s [speed]
```

### Input Parameters
- **filename**
  
  **Description:** Path to the log file.  
  **Example:**  
  `my_log_file.log`

- **maze**
  
  **Description:** Choose the type of maze. You can select one of the following:
  - square
  - cp1

- **speed**
  
  **Description:** Choose the speed setting for the maze. You can select one of the following:
  - low
  - high

## To Plot Particle Filter Update Time
```bash
$ python3 plot_PF_update_time.py
```

## To Plot SLAM Pose an Particles at a Regular Time Interval
```bash
$ python3 plot_SLAM_pose_and_particle.py -f [path/to/logFile]
```

## To Plot Odometry Pose and Body Velocity Driving in Square
```bash
$ python3 plot_drive_square.py -f [path/to/logFile]
```

## To Plot SLAM Trajectory vs Ground Truth and Analysis
```bash
$ python3 read_slam_stats.py -f [path/to/logFile]
```

## To Plot Motor Calibration Data
```bash
$ python3 plot_motor_calibration.py
