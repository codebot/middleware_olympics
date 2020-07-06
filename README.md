# Middleware Olympics

Middlewares are complex. Middleware comparisons are complex. There are
many dimensions of tradeoffs that go into the design and implementation
of middlewares, so any evaluation needs to consider many different
use cases. The goal of this repo is to have a bunch of little programs
to experiment with various middlewares to learn more about their pros and
cons.

### The Contestants

 * Zenoh
 * ZeroMQ (`ign_transport`)
 * DDS
   * Fast-RTPS (ROS 2)
   * Cyclone DDS (ROS 2)

### The Events

| Event Name | Description | Robot Application | Evaluation Metric |
| --- | --- | --- | --- |
| Hello world | Send strings from A to B. | Nav targets | Pass/fail |
| 1KB pub/sub | Send 1 KB messages at 1 kHz | Arm control | Latency and jitter |
| 1MB pub/sub | Send 1 MB messages at 30 Hz | Raw image stream | Latency and jitter |
| 10MB pub/sub | Send 10 MB messages at 10 Hz | Raw point clouds | Latency and jitter |
| Good WiFi pub/sub | Various messages over ideal WiFi | Mobile robots | Latency and jitter |
| Bad WiFi pub/sub | Various messages over intermittent WiFi | Mobile robots | Latency, jitter, drop rate, reconnect speed |
| Discovery | Discover various (large) numbers of topics | Startup | Discovery time |

# Holding the Middleware Olympics

Refer to the `README.md` file in the directory for each contestant for how to set up and run the events for that contestant.
