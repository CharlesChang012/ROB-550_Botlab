'''
Author: Po-Hsun Chang
Email: pohsun@umich.edu
Latest update date: 04/20/2025
'''

import sys
import lcm
import argparse
import matplotlib.pyplot as plt
from mbot_lcm_msgs import pose2D_t
from mbot_lcm_msgs import twist2D_t
import numpy as np

parser = argparse.ArgumentParser()
# Add parameters (arguments)
parser.add_argument('-f', '--file', type=str, help="Log file to read and plot", required=True)
args = parser.parse_args()


# Ground truth
x_gt = [0.0, 1.0, 1.0, 0.0, 0.0]
y_gt = [0.0, 0.0, 1.0, 1.0, 0.0]

# Check if the user has provided a log file as an argument
if len(sys.argv) < 2:
    sys.stderr.write("usage: python3 plot_drive_square.py -f [PathToLogFile]\n")
    sys.exit(1)

# Open the event log file in read mode
log = lcm.EventLog(args.file, "r")

# pose values in log files
x = []
y = []
theta = []

time = []
vx = []
wz = []

# Iterate through all events in the log
for event in log:
    # Check if the event is on the "MBOT_ODOMETRY" channel
    if event.channel == "MBOT_ODOMETRY":
        # Decode the data from the event
        msg = pose2D_t.decode(event.data)
        x.append(msg.x)
        y.append(msg.y)
        theta.append(msg.theta)

    if event.channel == "MBOT_VEL":
        # Decode the data from the event
        msg = twist2D_t.decode(event.data)
        time.append(msg.utime)
        vx.append(msg.vx)
        wz.append(msg.wz)
       
print(len(vx))
plt.figure(figsize=(6, 6))

# Plot the trajectory of the robot
plt.plot(x, y, marker='o', linestyle='-', color='b', label="Trajectory")

# Plot the ground truth of the path
plt.plot(x_gt, y_gt, linestyle='--', color='r', label="Ground truth")

plt.title('Robot Trajectory Running in Square')

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)

# Optional: Add a legend
plt.legend()

# Show the plot
plt.show()


# Create a new figure for velocity plots
plt.figure(figsize=(10, 5))

# Plot vx (linear velocity) with respect to time
plt.plot(time[:465], vx[:465], linestyle='-', color='g', label="Linear Velocity (vx)")
plt.title('Linear Velocitiy Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Linear Velocity (m/s)')
plt.grid(True)
plt.legend()
plt.show()

# # Plot wz (angular velocity) with respect to time
plt.figure(figsize=(10, 5))
plt.title('Angular Velocity Over Time')
plt.plot(time[:465], wz[:465], linestyle='-', color='m', label="Angular Velocity (wz)")
plt.xlabel('Time (s)')
plt.ylabel("Angular Velocity (rad/s)")
plt.grid(True)
plt.legend()
plt.show()