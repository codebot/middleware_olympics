#!/bin/bash
set -o errexit

if [ $# -lt 1 ]; then
  echo "usage: compile.sh MIDDLEWARE_NAME"
  echo "  example: compile.sh ros1"
  exit
fi

MIDDLEWARE_NAME=$1

if [ $MIDDLEWARE_NAME = ros1 ]; then
  echo "Building the ROS 1 contestants..."
  mkdir -p build/ros1/src
  ln -sf `pwd`/ros1/ros1_contestant build/ros1/src
  cd build/ros1
  source /opt/ros/melodic/setup.bash
  catkin build

elif [ $MIDDLEWARE_NAME = ros2 ]; then
  echo "Building the ROS 2 contestants...."
  mkdir -p build/ros2/src
  ln -sf `pwd`/ros2/ros2_contestant build/ros2/src
  cd build/ros2
  source /opt/ros/eloquent/setup.bash
  colcon build

elif [ $MIDDLEWARE_NAME = zenoh ]; then
  echo "Building the Zenoh contestants..."
  mkdir -p build/zenoh
  if [ ! -d build/zenoh/zenoh_contestant ]; then
    ln -s `pwd`/zenoh/zenoh_contestant build/zenoh
  fi
  cd build/zenoh/zenoh_contestant
  mkdir -p build
  cd build
  cmake ..
  make

else
  echo "unknown middleware: $MIDDLEWARE_NAME"
fi
