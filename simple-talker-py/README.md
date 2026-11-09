# ROS 2 Simple Talker (Publisher) (Python)

## Introduction

This project implements a simple ROS 2 talker, derived from [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python).

The differences to this template project are:

* ROS 2 runtime functions are provided by the `ros2-base` snap, which must be preinstalled on the ctrlX.
* The project files are packed into a snap.

See also:

* [Canonical Snapcraft - Deploying robotics applications](https://snapcraft.io/docs/robotics)
* [Canonical - How to build a snap using ROS 2 Jazzy](https://canonical.com/blog/how-to-build-a-snap-using-ros-2-jazzy)

## Implementation

* The directory structure is according colcon packages.
* In `setup.py` and `setup.cfg` the [colcon](https://colcon.readthedocs.io/en/released/) package is defined.
* The one and only Python script is `main_minimal_subscriber.py`, stored in the directory `src/listener/listener/`
* It contains the main function and a class `MinimalSubscriber`, derived from `rclpy.node.Node`
* `snap/snapcraft.yaml` defines which files are packed into the snap.

## Building a snap

Building a snap with:

```bash
    ./build-snap-amd64.sh
```

## About

SPDX-FileCopyrightText: Copyright (c) 2026 Bosch Rexroth AG

<https://www.boschrexroth.com/en/dc/imprint/>

## Licenses

SPDX-License-Identifier: MIT
