# ROS 2 Simple Listener (Python)

## Introduction

This project implements a simple ROS 2 listener, derived from [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python).

The differences to this template project are:

* ROS 2 runtime functions are provided by the `ros2-base` snap, which must be preinstalled on the ctrlX.
* The project files are packed into a snap.

See also:

* [Canonical Snapcraft - Deploying robotics applications](https://snapcraft.io/docs/robotics)
* [Canonical - How to build a snap using ROS 2 Humble](https://canonical.com/blog/how-to-build-a-snap-using-ros-2-humble)

## Implementation

* The directory structure is according colcon packages.
* In `setup.py` and `setup.cfg` the [colcon](https://colcon.readthedocs.io/en/released/) package is defined.
* The one and only Python script is `main_minimal_subscriber.py`, stored in the directory `src/listener/listener/`
* It contains the main function and a class `MinimalSubscriber`, derived from `rclpy.node.Node`
* `snap/snapcraft.yaml` defines which files are packed into the snap.

## Build Process

To build a snap run

    ./build-snap-amd64.sh

The script contains following build steps:

1. Build the colcon package:

    colcon build

2. Clean the snapcraft helper directories:

    snapcraft clean --destructive-mode

3. Update helper directories and build the snap:

    snapcraft --destructive-mode

Hint: To keep the project simple the arm64-cross-build is not supported.

## Installation and Test

Following snaps must already be installed:

* `ros2-base`
* `ros2-simple-talker`

Install the created snap on a ctrlX OS.

It listens at the ROS 2 topic `MinimalPublisher`.

The output can be checked with:

    sudo snap logs -f ros2-simple-listener

## Build, install and test

With following command you can combine the build, installation and test steps - here the target is a ctrlX CORE<sup>virtual</sup> with port forwarding.

    ../../../public/scripts/build-upload-log-snap.sh -PF

## About

SPDX-FileCopyrightText: Copyright (c) 2023 Bosch Rexroth AG

<https://www.boschrexroth.com/en/dc/imprint/>

## Licenses

SPDX-License-Identifier: MIT
