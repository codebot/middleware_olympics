#!/bin/bash
set -o errexit

echo "Building the ROS 1 contestant..."
mkdir -p build/ros1/src
ln -sf `pwd`/ros1/ros1_contestant build/ros1/src
pushd build/ros1
source /opt/ros/melodic/setup.bash
catkin build
popd


echo "Building the ROS 2 contestant..."
mkdir -p build/ros2/src
ln -sf `pwd`/ros2/ros2_contestant build/ros2/src
pushd build/ros2
source /opt/ros/eloquent/setup.bash
colcon build
popd
