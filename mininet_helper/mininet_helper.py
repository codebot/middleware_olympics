def parse_ifconfig_output(output):
    ifaces = output.split('\n\n')
    iface = [iface for iface in ifaces if not iface.startswith('lo')][0]
    lines = [line.strip() for line in iface.split('\n')]

    rx_line = [line for line in lines if line.startswith('RX packets')][0]
    rx_line_tokens = rx_line.split()
    rx_packets = int(rx_line_tokens[2])
    rx_bytes = int(rx_line_tokens[4])

    tx_line = [line for line in lines if line.startswith('TX packets')][0]
    tx_line_tokens = tx_line.split()
    tx_packets = int(tx_line_tokens[2])
    tx_bytes = int(tx_line_tokens[4])

    return (tx_packets, tx_bytes, rx_packets, rx_bytes)
