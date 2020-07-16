#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np

if __name__=='__main__':
    ros1 = np.genfromtxt('ros1.csv', delimiter=',')
    cyclone = np.genfromtxt('cyclone.csv', delimiter=',')
    fastrtps = np.genfromtxt('fastrtps.csv', delimiter=',')

    plt.loglog(ros1[:, 0], ros1[:, 2], marker='o', markersize=5)
    plt.loglog(fastrtps[:, 0], fastrtps[:, 2], marker='o', markersize=5)
    plt.loglog(cyclone[:, 0], cyclone[:, 2], marker='o', markersize=5)

    plt.title('Latency vs Message Size')
    plt.ylabel('Median latency (seconds)')
    plt.xlabel('Message payload size (bytes)')

    plt.legend(
      ('ROS 1', 'FastRTPS', 'Cyclone'),
      loc='upper left')

    plt.show()
