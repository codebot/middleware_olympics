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
| Good WiFi pub/sub | Various message over ideal WiFi | Mobile robots | Latency and jitter |
| Bad WiFi pub/sub | Various message over intermittent WiFi | Mobile robots | Latency, jitter, drop rate, reconnect speed |

# Setting up

### Zenoh

https://github.com/eclipse-zenoh/zenoh/tree/rust-master

It seems that Zenoh needs the very latest rust, rather than a version of
rust from a distribution package manager that may be somewhat older. So
if you have Rust already installed in Ubuntu, you'll need to do this first:
```
sudo apt remove rustc
```

Now, install the latest rust using its official installer:
```
mkdir -p ~/olympics/rust
cd ~/olympics/rust
wget https://sh.rustup.rs -O ./rustup-init
chmod +x ./rustup-init
./rustup-init
rustup install nightly
```

Now if you close the current shell and open a new one, this should work:
```
which rustc
```
should be something like `/home/USERNAME/.cargo/bin/rustc`

Now we can install the development branch of Zenoh:
```
mkdir ~/olympics
cd ~/olympics
git clone https://github.com/eclipse-zenoh/zenoh -b rust-master
cd zenoh
cargo +nightly build --release --all-targets
```
