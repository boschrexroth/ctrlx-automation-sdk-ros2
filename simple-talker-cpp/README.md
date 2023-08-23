# Simple ROS 2 Publisher in C++

ROS 2 node which publishes messages under the topic `ros2_simple_cpp`.

## Prerequisites

* `ros-base` snap. The base snap which provides the ROS 2 runtime binaries. Has to be installed on ctrlX OS. See [ROS 2 Humble Base Snap](../ros2-base-humble-deb/README.md).
* An Ubuntu based build environment to build an app. See [ctrlX Automation SDK](https://github.com/boschrexroth/ctrlx-automation-sdk).

## Basis for this Project

This project is based on the official ROS 2 Tutorial: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c).

## Building a snap

Building a snap has two steps:

1. Colcon build: CMakeLists.txt defines how compile and link the C++ sources.
2. snap build: snap/snapcraft.yaml defines how the compiled binaries are packed into the snap and how they are called on the ctrlX CORE.

### Colcon Configuration

The colcon build tool is configured by `CMakeLists.txt`.

This section defines the ROS 2 packages needed:

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

And here the executables and their dependencies are defined:

    add_executable(listener src/subscriber_member_function.cpp)
    ament_target_dependencies(listener rclcpp std_msgs)

### Snapcraft Configuration

`snap/snapcraft.yaml` defines how the snap will be build:

* `install/` is dumped into the snap
* also `wrapper/`
* Two apps (talker and listener) are copied into the snap and started as services
* The snap - respectively the executables - uses the content interface of the `ros-base` snap (here the ROS 2 runtime is provided).

### Build the snap

Start this script:

    ./build-snap-amd64.sh

## About

SPDX-FileCopyrightText: Copyright (c) 2023 Bosch Rexroth AG

<https://www.boschrexroth.com/en/dc/imprint/>

## Licenses

SPDX-License-Identifier: Apache-2.0
