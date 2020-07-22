#!/bin/bash
set -o errexit

if [ $# -lt 1 ]; then
  echo "usage: dependencies.sh MIDDLEWARE_NAME"
  echo "  example: dependencies.sh zenoh"
  exit
fi

MIDDLEWARE_NAME=$1

if [ $MIDDLEWARE_NAME = zenoh ]; then
  echo "Setting up dependencies of the Zenoh contestants..."
  mkdir -p build/zenoh
  cd build/zenoh

  if [ ! -f ./rustup-init ]; then
    wget https://sh.rustup.rs -O ./rustup-init
    chmod +x ./rustup-init
    ./rustup-init
    rustup install nightly
  fi

  if [ ! -d zenoh ]; then
    git clone https://github.com/eclipse-zenoh/zenoh -b rust-master
  else
    pushd zenoh
    git pull
    popd
  fi

  cd zenoh
  cargo +nightly build --release --all-targets
  cargo +nightly install cbindgen

  cd zenoh-ffi
  ./gencbind.sh

else
  echo "unknown middleware: $MIDDLEWARE_NAME"
fi

