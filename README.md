# Middleware Olympics

Middlewares are complex. Middleware comparisons are complex. There are
many dimensions of tradeoffs that go into the design and implementation
of middlewares, so any evaluation needs to consider many different
use cases. The goal of this repo is to have a bunch of little programs
to experiment with various middlewares to learn more about their pros and
cons.

## Contestants

 * Zenoh
 * ZeroMQ (`ign_transport`)
 * DDS
   * Fast-RTPS (ROS 2)
   * Cyclone DDS (ROS 2)

# Setting up

### Zenoh

https://github.com/eclipse-zenoh/zenoh/tree/rust-master

First install rust:
```
mkdir -p ~/olympics/rust
cd ~/olympics/rust
wget https://sh.rustup.rs -O ./rustup
chmod +x ./rustup
```

```
mkdir ~/olympics
cd ~/olympics
git clone https://github.com/eclipse-zenoh/zenoh -b rust-master
cd zenoh
cargo build --release --all-targets
```
