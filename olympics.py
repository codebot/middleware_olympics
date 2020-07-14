#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import time

def run_ros1_pubsub(message_count, blob_size, rate):
    subscriber = subprocess.Popen(
        [
            '/bin/bash',
            '-c',
            f'. build/ros1/devel/setup.bash && rosrun ros1_contestant subscriber _max_message_count:={message_count} _tcp_nodelay:=true'],
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


def run_ros1():
    print('starting roscore...')
    roscore = subprocess.Popen(
        ['/bin/bash', '-c', '. build/ros1/devel/setup.bash && roscore'],
        preexec_fn=os.setsid)

    print('wait a bit for roscore to start up...')
    time.sleep(2)

    payloads_and_rates = [
        [10, 10],
        [10, 100],
        [10, 1000],
        [1000, 10],
        [1000, 100],
        [1000, 1000],
        [10000, 10],
        [10000, 100],
        [10000, 1000],
        [100000, 10],
        [100000, 100],
        [1000000, 10],
        [1000000, 100],
        [10000000, 10]
    ]

    for experiment in payloads_and_rates:
        payload = experiment[0]
        rate = experiment[1]
        stats = run_ros1_pubsub(rate * 10, payload, rate)
        latency_avg = stats[0]
        latency_min = stats[1]
        latency_max = stats[2]
        skips = stats[3]
        print(f'{payload} {rate} {latency_avg} {latency_min} {latency_max} {skips}')

    print('killing roscore...')
    try:
        os.killpg(os.getpgid(roscore.pid), signal.SIGINT)
        roscore.wait(timeout=1)
    except ProcessLookupError:
        pass

def run_fastdds():
    print('fastdds')

def run_cyclone():
    print('cyclone')

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("contestant")
    args = parser.parse_args()
    if args.contestant == 'ros1':
        run_ros1()
    elif args.contestant == 'cyclone':
        run_cyclone()
    elif args.contestant == 'fastdds':
        run_fastdds()
    else:
        print(f'unknown contestant: {args.contestant}')
