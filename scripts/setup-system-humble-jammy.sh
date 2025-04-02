#!/bin/bash

set -e

# see: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
echo "Setup ROS2 Humble environment"

#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#  +++++++++ Set locale  +++++++++
#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
echo "-- Set locale"
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#  +++++++++ Setup Sources  +++++++++
#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
echo "-- Add the ROS 2 apt repository"

sudo apt install software-properties-common
sudo add-apt-repository universe -y

## Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

## Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++ Install ROS 2 packages +++++++++
#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
echo "-- Install ROS 2 packages"
sudo apt update && sudo apt upgrade -y

## ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.
sudo apt install -y ros-humble-ros-base

## Development tools: Compilers and other tools to build ROS packages
sudo apt install -y ros-dev-tools

