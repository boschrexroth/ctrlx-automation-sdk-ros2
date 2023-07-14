# Ros2 Simple Talker 

## Introduction

This project implements a simple ROS 2 talker, derived from [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python). 

The differences to this template project are:

* __ROS 2 runtime functions are provided by the ros2-base snap, which must be preinsalled on the ctrlX.__
* The project files are packed into a snap.

See also:

* [Canonical Snapcraft - Deploying robotics applications](https://snapcraft.io/docs/robotics)
* [Canonical - How to build a snap using ROS 2 Humble](https://canonical.com/blog/how-to-build-a-snap-using-ros-2-humble)

## Implementation

* The directory structure is according colcon packages.
* In setup.py and setup.cfg the [colcon](https://colcon.readthedocs.io/en/released/) package is defined.
* The one and only Python script is main_minimal_publisher.py__, stored in the directory src/listener/listener/
* It contains the main function and a class __MinimalPublisher__, derived from rclpy.node.Node
* snap/snapcraft.yaml defines which files are packed into the snap. 

## Build process

To build a snap run

    ./build-snap-amd64.sh

The script contains following build steps:

1. Build the colcon package: 

    colcon build

2. Clean the snapcraft helper directories: 

    snapcraft clean --destructive-mode

3. Update helper directories and build the snap:

    snapcraft --destructive-mode

Hint: To keep the project simple arm64-cross-build is not supported.

## Installation and test

The ros2-base snaps must already be installed.

Install the created snap on a ctrlX CORE (virtual or X7).

It publishes at the ROS 2 topic 'MinimalPublisher'.

The output can be checked with:

    sudo snap logs -f ros2-simple-talker

## Build, install and test

With following command you can combine the build, installation and test steps - here the target is a ctrlX CORE<sup>virtual</sup> with Port Forwarding.

    ../../../public/scripts/build-upload-log-snap.sh -PF


## About

Copyright © 2023 Bosch Rexroth AG. All rights reserved.

<https://www.boschrexroth.com>

Bosch Rexroth AG  
Bgm.-Dr.-Nebel-Str. 2  
97816 Lohr am Main  
GERMANY  

## Licenses

SPDX-FileCopyrightText: Bosch Rexroth AG
SPDX-License-Identifier: MIT
