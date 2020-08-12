#!/usr/bin/env python3

from mininet_helper import parse_ifconfig_output

parse_ifconfig_output('''
h1-eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.0.0.1  netmask 255.0.0.0  broadcast 10.255.255.255
        ether 72:dd:75:f2:44:65  txqueuelen 1000  (Ethernet)
        RX packets 27  bytes 1808 (1.8 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 25  bytes 1636 (1.6 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
''')
