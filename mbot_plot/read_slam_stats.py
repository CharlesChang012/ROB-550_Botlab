'''
Author: Po-Hsun Chang
Email: pohsun@umich.edu
Latest update date: 04/16/2025
'''

import sys
import lcm
import argparse
import matplotlib.pyplot as plt
from mbot_lcm_msgs import pose2D_t
import numpy as np

parser = argparse.ArgumentParser()
# Add parameters (arguments)
parser.add_argument('-f', '--file', type=str, help="Log file to read and plot", required=True)
args = parser.parse_args()

# Check if the user has provided a log file as an argument
if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

# Open the event log file in read mode
log = lcm.EventLog(args.file, "r")

# pose values in log files
x = []
y = []
theta = []
utime = []

gt_x = []
gt_y = []
gt_theta = []
gt_utime = []

# Iterate through all events in the log
for event in log:
    # Check if the event is on the "MBOT_ODOMETRY" channel
    if event.channel == "SLAM_POSE":
        # Decode the data from the event
        msg = pose2D_t.decode(event.data)
        x.append(msg.x)
        y.append(msg.y)
        theta.append(msg.theta)
        utime.append(msg.utime)

    if event.channel == "GROUND_TRUTH_POSE":
        # Decode the data from the event
        msg = pose2D_t.decode(event.data)
        gt_x.append(msg.x)
        gt_y.append(msg.y)
        gt_theta.append(msg.theta)
        gt_utime.append(msg.utime)

        # Interpolate ground truth positions at specific utime
        matched_gt_x = np.interp(utime, gt_utime, gt_x)
        matched_gt_y = np.interp(utime, gt_utime, gt_y)

print(len(gt_x))
plt.figure(figsize=(8, 6))

# Plot the trajectory of the robot
plt.plot(x, y, linestyle='-', color='b', label="SLAM Pose")

# Plot the ground truth of the path
plt.plot(matched_gt_x, matched_gt_y, linestyle='--', color='r', label="Ground Truth")

# Calculate RMS error
rms_error_x = np.sqrt(np.mean((np.array(matched_gt_x) - np.array(x))**2))
rms_error_y = np.sqrt(np.mean((np.array(matched_gt_y) - np.array(y))**2))
# Calculate standard deviation
std_dev_x = np.std(np.array(matched_gt_x) - np.array(x))
std_dev_y = np.std(np.array(matched_gt_y) - np.array(y))

# Display RMS error on the plot
plt.title(f"Slam Pose vs Ground Truth")

# Display standard deviation on the plot
plt.text(0.98, 0.02, f"RMS Error x: {rms_error_x:.4f}\nRMS Error y: {rms_error_y:.4f}\nStd Dev x: {std_dev_x:.4f}\nStd Dev y: {std_dev_y:.4f}\n", 
         transform=plt.gca().transAxes, fontsize=10, verticalalignment='bottom', horizontalalignment='right')

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.grid(True)
plt.legend()

# Show the plot
plt.show()