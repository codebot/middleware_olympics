#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import time

payloads_and_rates = [
    [1, 500],
    [10, 500],
    [20, 500],
    [40, 500],
    [100, 500],
    [200, 500],
    [400, 500],
    [1000, 500],
    [2000, 500],
    [4000, 500],
    [10000, 100],
    [20000, 100],
    [40000, 100],
    [100000, 100],
    [200000, 100],
    [400000, 100],
    [1000000, 30],
    [2000000, 30],
    [4000000, 30],
    [10000000, 10],
    [20000000, 10],
    [40000000, 5],
]


def run_ros2_pubsub(rmw, message_count, blob_size, rate):
    subscriber = subprocess.Popen(
        [
            '/bin/bash',
            '-c',
            f'RMW_IMPLEMENTATION={rmw} . build/ros2/install/setup.bash && ros2 run ros2_contestant subscriber --ros-args -p max_message_count:={message_count}'],
        preexec_fn=os.setsid,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)

    time.sleep(1)

    # send another 1 second of messages to cover the connection time
    pub_message_count = message_count + 2 * int(rate)

    publisher = subprocess.Popen(
        [
            '/bin/bash',
            '-c',
            f'RMW_IMPLEMENTATION={rmw} . build/ros2/install/setup.bash && ros2 run ros2_contestant publisher --ros-args -p max_message_count:={pub_message_count} -p publish_rate:={rate:.2f} -p blob_size:={blob_size}'],
        preexec_fn=os.setsid)

    expected_time = 4 + message_count / rate
    # print(f'waiting {expected_time} seconds for the event...')
    time.sleep(expected_time)

    subscriber_stdout, subscriber_stderr = subscriber.communicate()
    stats = subscriber_stdout.split()

    try:
        os.killpg(os.getpgid(subscriber.pid), signal.SIGINT)
        subscriber.wait(timeout=1)
    except ProcessLookupError:
        pass

    try:
        os.killpg(os.getpgid(publisher.pid), signal.SIGINT)
        publisher.wait(timeout=1)
    except ProcessLookupError:
        pass

    return [float(stats[0]), float(stats[1]), float(stats[2]), int(stats[3])]


def run_ros1_pubsub(message_count, blob_size, rate, tcp_nodelay):
    subscriber = subprocess.Popen(
        [
            '/bin/bash',
            '-c',
            f'. build/ros1/devel/setup.bash && rosrun ros1_contestant subscriber _max_message_count:={message_count} _tcp_nodelay:={str(tcp_nodelay).lower()}'],
        preexec_fn=os.setsid,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)

    time.sleep(1)

    # send another 1 second of messages to cover the connection time
    pub_message_count = message_count + 2 * int(rate)

    publisher = subprocess.Popen(
        [
            '/bin/bash',
            '-c',
            f'. build/ros1/devel/setup.bash && rosrun ros1_contestant publisher _max_message_count:={pub_message_count} _publish_rate:={rate} _blob_size:={blob_size}'],
        preexec_fn=os.setsid)

    expected_time = 2 + message_count / rate
    # print(f'waiting {expected_time} seconds for the event...')
    time.sleep(expected_time)

    subscriber_stdout, subscriber_stderr = subscriber.communicate()
    stats = subscriber_stdout.split()

    try:
        os.killpg(os.getpgid(subscriber.pid), signal.SIGINT)
        subscriber.wait(timeout=1)
    except ProcessLookupError:
        pass

    try:
        os.killpg(os.getpgid(publisher.pid), signal.SIGINT)
        publisher.wait(timeout=1)
    except ProcessLookupError:
        pass

    return [float(stats[0]), float(stats[1]), float(stats[2]), int(stats[3])]


def run_ros1(tcp_nodelay):
    print('starting roscore...')
    roscore = subprocess.Popen(
        ['/bin/bash', '-c', '. build/ros1/devel/setup.bash && roscore'],
        preexec_fn=os.setsid)

    print('wait a bit for roscore to start up...')
    time.sleep(2)

    f = open(f'ros1_{tcp_nodelay}.csv', 'w')
    for experiment in payloads_and_rates:
        payload = experiment[0]
        rate = experiment[1]
        stats = run_ros1_pubsub(rate * 10, payload, rate, tcp_nodelay)
        latency_avg = stats[0]
        latency_min = stats[1]
        latency_max = stats[2]
        skips = stats[3]
        print(f'{payload} {rate} {latency_avg} {latency_min} {latency_max} {skips}')
        f.write(f'{payload},{rate},{latency_avg},{latency_min},{latency_max},{skips}\n')
        f.flush()

    print('killing roscore...')
    try:
        os.killpg(os.getpgid(roscore.pid), signal.SIGINT)
        roscore.wait(timeout=1)
    except ProcessLookupError:
        pass

def run_fastrtps():
    f = open('fastrtps.csv', 'w')
    for experiment in payloads_and_rates:
        payload = experiment[0]
        rate = experiment[1]
        stats = run_ros2_pubsub('rmw_fastrtps_cpp', rate * 10, payload, rate)
        latency_avg = stats[0]
        latency_min = stats[1]
        latency_max = stats[2]
        skips = stats[3]
        print(f'{payload} {rate} {latency_avg} {latency_min} {latency_max} {skips}')
        f.write(f'{payload},{rate},{latency_avg},{latency_min},{latency_max},{skips}\n')
        f.flush()

def run_cyclone():
    f = open('cyclone.csv', 'w')
    for experiment in payloads_and_rates:
        payload = experiment[0]
        rate = experiment[1]
        stats = run_ros2_pubsub('rmw_cyclonedds_cpp', rate * 10, payload, rate)
        latency_avg = stats[0]
        latency_min = stats[1]
        latency_max = stats[2]
        skips = stats[3]

        print(f'{payload} {rate} {latency_avg} {latency_min} {latency_max} {skips}')
        f.write(f'{payload},{rate},{latency_avg},{latency_min},{latency_max},{skips}\n')
        f.flush()


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("contestant")
    args = parser.parse_args()
    if args.contestant == 'ros1':
        run_ros1(False)
    elif args.contestant == 'ros1_tcpnodelay':
        run_ros1(True)
    elif args.contestant == 'cyclone':
        run_cyclone()
    elif args.contestant == 'fastrtps':
        run_fastrtps()
    else:
        print(f'unknown contestant: {args.contestant}')
