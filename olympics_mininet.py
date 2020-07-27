#!/usr/bin/env python3

"""
This file is an experiment towards using Mininet to play around with
various ways to instrument middleware discovery and transport.

Acknowldgement: initially following along the work of Jacob Perron:
https://github.com/jacobperron/mininet_ros_demo
"""

from argparse import ArgumentParser
import time

from mininet.log import setLogLevel
from mininet.net import Mininet
from mininet.topo import Topo
from mininet.util import dumpNodeConnections


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        '--num_hosts',
        default=2,
        type=int,
        help='number of hosts'
    )
    args = parser.parse_args()

    print(f'running on {args.num_hosts} hosts')
    topo = Topo()
    switch = topo.addSwitch('s1')
    for i in range(1, args.num_hosts + 1):
        host = topo.addHost(f'h{i}')
        topo.addLink(host, switch)
    net = Mininet(topo)
    net.start()
    dumpNodeConnections(net.hosts)
    net.pingAll()
    time.sleep(20)


    outputs = [''] * len(net.hosts)

    for host in net.hosts:
        host.sendCmd('ifconfig | grep "RX packet"')

    for i, host in enumerate(net.hosts):
        while host.waiting:
            #print(f'about to monitor host {host.name}')
            text = host.monitor(10)
            outputs[i] += text
            #print(f'output: {text}')

    for host, output in zip(net.hosts, outputs):
        print(f'host {host.name}:\n{output}\n')

    net.stop()
