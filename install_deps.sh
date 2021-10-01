#!/usr/bin/env bash

set -e -x

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo  apt-add-repository -y "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"
export ROS_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
sudo apt-get install -y ros-$ROS_DISTRO-desktop-full
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y libeigen3-dev python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
