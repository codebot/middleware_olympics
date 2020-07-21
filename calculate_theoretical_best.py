#!/usr/bin/env python3

payloads = [
    100, 200, 400,
    1000, 2000, 4000,
    10000, 20000, 40000,
    100000, 200000, 400000,
    1000000, 2000000, 4000000,
    10000000
]

f = open('theoretical_limit.csv', 'w')
for payload in payloads:
    min_latency = 2 * (payload) * 8 / 1.0e9
    f.write(f'{payload},1,{min_latency},{min_latency},{min_latency},0\n')
