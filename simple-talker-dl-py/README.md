# Ros2 Talker of ctrlX Data Layer value

## Introduction

This project implements a ROS 2 talker, which reads a ctrlX Data Layer value and publishes it as ROS 2 message.

[Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python) was used as template. 

See also:

* [Canonical Snapcraft - Deploying robotics applications](https://snapcraft.io/docs/robotics)
* [Canonical - How to build a snap using ROS 2 Humble](https://canonical.com/blog/how-to-build-a-snap-using-ros-2-humble)

## Implementation

* The directory structure is according colcon packages.
* In setup.py and setup.cfg the [colcon](https://colcon.readthedocs.io/en/released/) package is defined.
* src/talker/talker/main.py provides the main function
* src/talker/ros2/datalayer_reader_ros2_publisher.py reads a ctrlX Data LAyer values, creates a ROS 2 message and publishes it under the topic 'ctrlXCpuUtilisationPercent'.
* src/talker/ctrlx_datalayer/ctrlx_datalayer_helper.py contains helper functions regarding ctrlX Data Layer
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

Hint: Arm64-cross-build is not supported.

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
