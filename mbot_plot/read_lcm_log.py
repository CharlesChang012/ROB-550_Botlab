'''
Author: Po-Hsun Chang
Email: pohsun@umich.edu
Latest update date: 05/20/2025
'''

import sys
import lcm
import argparse
import matplotlib.pyplot as plt
from mbot_lcm_msgs import pose2D_t

parser = argparse.ArgumentParser()
# Add parameters (arguments)
parser.add_argument('-m', '--map', type=str, help="Map: square or cp1", required=True)
parser.add_argument('-f', '--file', type=str, help="Log file to read and plot", required=True)
parser.add_argument('-s', '--speed', type=str, help="high or low speed", required=False)
args = parser.parse_args()

if args.map == "square":
    x_bound = [-0.5, 1.5, 1.5, -0.5, -0.5]
    y_bound = [-0.5, -0.5, 1.5, 1.5, -0.5]

    pose_start = [0.0, 0.0]
    pose_goal = [0.0, 0.0]

    # Ground truth
    x_gt = [0.0, 1.0, 1.0, 0.0, 0.0]
    y_gt = [0.0, 0.0, 1.0, 1.0, 0.0]

elif args.map == "cp1":
    ## User defined variables
    # CP1 maze boundary
    x_bound = [-0.305, -0.305, 0.305, 0.305, 1.525, 1.525, 1.525, 2.745, 2.745, 3.355, 3.355, 2.135, 2.135, 2.135, 2.135, 0.915, 0.915, 0.915, -0.305]
    y_bound = [0.305, -0.305, -0.305, -0.915, -0.915, 0.305, -0.915, -0.915, -0.305, -0.305, 0.305, 0.305, -0.305, 0.305, 0.915, 0.915,  -0.305, 0.305, 0.305]

    # CP1 maze start and goal pose
    pose_start = [0, 0]
    pose_goal = [3.05, 0]

    # Ground truth
    x_gt = [0.0, 0.61, 0.61, 1.22, 1.22, 1.83, 1.83, 2.44, 2.44, 3.05]
    y_gt = [0.0, 0.0, -0.61, -0.61, 0.61, 0.61, -0.61, -0.61, 0.0, 0.0]

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

# Iterate through all events in the log
for event in log:
    # Check if the event is on the "MBOT_ODOMETRY" channel
    if event.channel == "MBOT_ODOMETRY":
        # Decode the data from the event
        msg = pose2D_t.decode(event.data)
        x.append(msg.x)
        y.append(msg.y)
        theta.append(msg.theta)
        # Print the decoded message
        print("Message:")
        print("   timestamp   = %s" % str(msg.utime))
        print("   x    = %s" % str(msg.x))
        print("   y = %s" % str(msg.y))
        print("   theta: %s" % str(msg.theta))
        print("")  # Blank line for readability

# # Open a file to write (it will create the file if it doesn't exist)
# with open("mbot_pose2D_drive_cp1_low_speed.txt", "w") as f:
#     for xi, yi in zip(x, y):
#         f.write(f"{xi} {yi}\n")  # Write each pair of coordinates on a new line

plt.figure(figsize=(8, 6))

# Plot the boundary of the maze
plt.plot(x_bound, y_bound, marker='.', linestyle='-', color='black', label="Boundary")

# Plot the trajectory of the robot
plt.plot(x, y, marker='o', linestyle='-', color='b', label="Trajectory")

# Plot the ground truth of the path
plt.plot(x_gt, y_gt, linestyle='--', color='r', label="Ground truth")

# Plot start and goal pose
plt.scatter(pose_start[0], pose_start[1],  marker='o', color='r', label='Start Pose', s = 150)
plt.scatter(pose_goal[0], pose_goal[1], marker='*', color='r', label='Goal Pose', s = 150)

# Optional: Add labels, title, and grid
if args.speed == "low":
    plt.title('Robot Trajectory at Low Speed')
elif args.speed == "high":
    plt.title('Robot Trajectory at High Speed')
else:
    plt.title('Robot Trajectory at Square')

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)

# Optional: Add a legend
plt.legend()

# Show the plot
plt.show()