#!/bin/bash

set -e

# see: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
echo "Setup ROS2 Jazzy environment"

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
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
export ROS_APT_SOURCE_VERSION
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++ Install ROS 2 packages +++++++++
#  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
echo "-- Install ROS 2 packages"
sudo apt update && sudo apt upgrade -y

## ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.
sudo apt install -y ros-jazzy-ros-base

## Development tools: Compilers and other tools to build ROS packages
sudo apt install -y ros-dev-tools

