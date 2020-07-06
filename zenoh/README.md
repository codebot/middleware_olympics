# Zenoh

### Installing

https://github.com/eclipse-zenoh/zenoh/tree/rust-master

It seems that Zenoh needs the very latest rust, rather than a version of rust from a distribution package manager that may be somewhat older.
So if you have Rust already installed in Ubuntu, you'll need to do this first:

```shell
sudo apt remove rustc
```

Now, install the latest rust using its official installer:

```shell
mkdir -p ~/olympics/rust
cd ~/olympics/rust
wget https://sh.rustup.rs -O ./rustup-init
chmod +x ./rustup-init
./rustup-init
rustup install nightly
```

Now if you close the current shell and open a new one, this should work:

```shell
which rustc
```

should be something like `/home/USERNAME/.cargo/bin/rustc`

Now we can install the development branch of Zenoh:

```shell
mkdir ~/olympics
cd ~/olympics
git clone https://github.com/eclipse-zenoh/zenoh -b rust-master
cd zenoh
cargo +nightly build --release --all-targets
```


# Documentation

http://zenoh.io/
