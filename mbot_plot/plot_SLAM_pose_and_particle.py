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
from mbot_lcm_msgs import particles_t
from mbot_lcm_msgs import particle_t

parser = argparse.ArgumentParser()
# Add parameters (arguments)
parser.add_argument('-f', '--file', type=str, help="Log file to read and plot", required=True)
args = parser.parse_args()

# Check if the user has provided a log file as an argument
if len(sys.argv) < 2:
    sys.stderr.write("usage: python3 plot_SLAM_pose_and_particle.py -f [PathToLogFile]\n")
    sys.exit(1)

# Open the event log file in read mode
log = lcm.EventLog(args.file, "r")

# pose values in log files
x = []
y = []
utime = []
particles = []
particles_x = []
particles_y = []

# Iterate through all events in the log
for event in log:
    # Check if the event is on the "MBOT_ODOMETRY" channel
    if event.channel == "SLAM_POSE":
        # Decode the data from the event
        msg = pose2D_t.decode(event.data)
        x.append(msg.x)
        y.append(msg.y)

    if event.channel == "SLAM_PARTICLES":
        msg = particles_t.decode(event.data)
        utime.append(msg.utime)
        particles.append(msg.particles)

        
plt.figure(figsize=(8, 6))

# Plot the trajectory of the robot
plt.plot(x, y, linestyle='-', color='b', label="SLAM Pose")

# Plot particles at a regular time rate
for i, particle_set in enumerate(particles):
    if i % 50 == 0:  # Adjust the rate by changing the modulus value
        for particle in particle_set:
            particles_x.append(particle.pose.x)
            particles_y.append(particle.pose.y)
            

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.scatter(particles_x, particles_y, color='r', s=1, alpha=0.5, label="Particles")
plt.title("Robot SLAM Pose and Particles")

plt.legend()
# Show the plot
plt.show()