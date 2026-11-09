# ROS 2 Talker (Publisher) of a ctrlX Data Layer Value (Python)

## Introduction

This project implements a ROS 2 talker, which reads a ctrlX Data Layer value and publishes it as ROS 2 message.

[Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python) was used as template.

See also:

* [Canonical Snapcraft - Deploying robotics applications](https://snapcraft.io/docs/robotics)
* [Canonical - How to build a snap using ROS 2 Jazzy](https://canonical.com/blog/how-to-build-a-snap-using-ros-2-jazzy)

## Implementation

* The directory structure is according colcon packages.
* In `setup.py` and `setup.cfg` the [colcon](https://colcon.readthedocs.io/en/released/) package is defined.
* `src/talker/talker/main.py` provides the main function
* `src/talker/ros2/datalayer_reader_ros2_publisher.py` reads a ctrlX Data Layer value, creates a ROS 2 message and publishes it under the topic `ctrlXCpuUtilisationPercent`.
* `src/talker/ctrlx_datalayer/ctrlx_datalayer_helper.py` contains helper functions regarding ctrlX Data Layer
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

SPDX-License-Identifier: Apache-2.0
