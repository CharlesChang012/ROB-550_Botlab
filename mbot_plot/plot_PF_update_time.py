'''
Author: Po-Hsun Chang
Email: pohsun@umich.edu
Latest update date: 04/11/2025
'''

import numpy as np
import matplotlib.pyplot as plt

particleNum = [100, 500, 1000, 4200, 10000, 30000, 50000]
timeToUpdate = [0.001, 0.006, 0.015, 0.1, 0.35, 2.350, 6.050]   # (s)

plt.figure(figsize=(8, 6))
plt.scatter(particleNum, timeToUpdate, marker='o', color='b', label='Data Points')
plt.axhline(y=0.1, color='grey', linestyle='--', label='y = 0.1')   # horizontal line at y=0.1 (10 Hz)

plt.title('Particle Filter Update Time vs Number of Particles', fontsize=14)
plt.xlabel('Number of Particles', fontsize=12)
plt.ylabel('Time to Update Particle Filter (s)', fontsize=12)
plt.ticklabel_format(axis='x', style='sci', scilimits=(4,4))

handles, labels = plt.gca().get_legend_handles_labels()
plt.legend(handles[1:], labels[1:])

plt.show()
