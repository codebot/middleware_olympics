#!/usr/bin/env python3

"""
This file is an experiment towards using Mininet to play around with
various ways to instrument middleware discovery and transport.

Acknowldgement: initially following along the work of Jacob Perron:
https://github.com/jacobperron/mininet_ros_demo
"""

from argparse import ArgumentParser
import time
import sys

from mininet.log import setLogLevel
from mininet.net import Mininet
from mininet.topo import Topo
from mininet.util import dumpNodeConnections

from mininet_helper.mininet_helper import parse_ifconfig_output


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
    prev_outputs = [(0, 0, 0, 0)] * args.num_hosts
    print(prev_outputs)

    net = Mininet(topo)

    # ipv6 seems to add a bunch of background network noise that we don't want
    for switch in net.switches:
        switch.cmd("sysctl -w net.ipv6.conf.all.disable_ipv6=1")
        switch.cmd("sysctl -w net.ipv6.conf.default.disable_ipv6=1")
        switch.cmd("sysctl -w net.ipv6.conf.lo.disable_ipv6=1")

    for host in net.hosts:
        host.cmd("sysctl -w net.ipv6.conf.all.disable_ipv6=1")
        host.cmd("sysctl -w net.ipv6.conf.default.disable_ipv6=1")
        host.cmd("sysctl -w net.ipv6.conf.lo.disable_ipv6=1")
        iface = f'{host.name}-eth0'
        host.cmd(f'route add -net 224.0.0.0 netmask 224.0.0.0 {iface}')

    #net.hosts[0].sendCmd('route -n')
    #print(f'host 0 name: {net.hosts[0].name}')

    net.start()
    dumpNodeConnections(net.hosts)
    #net.pingAll()
    print('waiting...')
    time.sleep(2)
    print('running stuff')

    f = open('zenoh_discovery.csv', 'a')

    zpath = './zenoh/zenoh_contestant/build'
    for num_sub_exp in range(0, 12):
        num_subs = 2**num_sub_exp
        print(f'num_subs: {num_subs}')

        for i, host in enumerate(net.hosts):
            if i == 0:
                host.sendCmd(f'{zpath}/discovery_publisher')
            else:
                host.sendCmd(f'{zpath}/discovery_subscriber {num_subs}')

        outputs = [''] * len(net.hosts)

        for i, host in enumerate(net.hosts):
            while host.waiting:
                # print(f'about to monitor host {host.name}')
                text = host.monitor(1000)
                outputs[i] += text
                # print(f'output {len(text)} bytes...')

        #for host, output in zip(net.hosts, outputs):
        #    print(f'host {host.name}:\n{output}\n')

        ###############################
        # now ask for the network usage
        ifconfig_outputs = [''] * len(net.hosts)

        for host in net.hosts:
            host.sendCmd('ifconfig')

        for i, host in enumerate(net.hosts):
            while host.waiting:
                #print(f'about to monitor host {host.name}')
                text = host.monitor(10)
                ifconfig_outputs[i] += text
                #print(f'output: {text}')

        host_idx = 0
        for host, output, prev in zip(net.hosts, ifconfig_outputs, prev_outputs):
            parsed_output = parse_ifconfig_output(output)
            print(f'host {host.name}: {parsed_output}')

            txp = parsed_output[0] - prev[0]
            txb = parsed_output[1] - prev[1]
            rxp = parsed_output[2] - prev[2]
            rxb = parsed_output[3] - prev[3]

            f.write(f'{len(net.hosts)},{num_subs},{txp},{txb},{rxp},{rxb}\n')
            f.flush()

            prev_outputs[host_idx] = parsed_output
            host_idx += 1

    net.stop()
