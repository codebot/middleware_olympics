#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    prefix = 'results/wired/'

    ros1_nagle = np.genfromtxt(prefix + 'ros1_False.csv', delimiter=',')
    ros1_tcp_nodelay = np.genfromtxt(prefix + 'ros1_True.csv', delimiter=',')
    cyclone = np.genfromtxt(prefix + 'cyclone.csv', delimiter=',')
    fastrtps = np.genfromtxt(prefix + 'fastrtps.csv', delimiter=',')
    limit = np.genfromtxt(prefix + 'theoretical_limit.csv', delimiter=',')

    lw = 2

    limit_start = 8
    plt.plot(limit[limit_start:, 0], limit[limit_start:, 2], marker='', markersize=5, color='black', linewidth=lw)

    plt.plot(ros1_nagle[:, 0], ros1_nagle[:, 2], marker='', markersize=5, linewidth=lw, color='blue')

    plt.plot(
        ros1_tcp_nodelay[:, 0],
        ros1_tcp_nodelay[:, 2],
        marker='',
        markersize=5,
        linewidth=lw)

    plt.plot(fastrtps[:, 0], fastrtps[:, 2], marker='', markersize=5, linewidth=lw)
    plt.loglog(cyclone[:, 0], cyclone[:, 2], marker='', markersize=5, linewidth=lw)

    plt.title('Round-trip time vs Message Size')
    plt.ylabel('Median round-trip time (seconds)')
    plt.xlabel('Message payload size (bytes)')

    plt.legend(
        (
            'Theoretical limit (payload only, at 1 Gbps)',
            'ROS 1 (default)',
            'ROS 1 (TCP_NODELAY)',
            'ROS 2 (FastRTPS)',
            'ROS 2 (Cyclone)'
        ),
        loc='upper left')

    plt.show()
