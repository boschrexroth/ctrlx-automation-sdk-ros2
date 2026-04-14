# Simple ROS 2 Remote Scheduler in C++

## Introduction

By implementing a scheduler callable, it is possible to run code in the realtime control cycle (e.g. every 1ms) and synchronized to other realtime services like the EtherCAT Master or the PLC realtime task. This is important for realtime control or to send joint setpoints to connected service drives.
Once registered, the callable will show up in the ctrlX OS scheduler settings overview.
To achieve this, you must:

* Instantiate a data layer system.
* Instantiate a remote scheduler system.
* Register a callable factory with the remote scheduler system.
* A simple ROS node is cyclically triggered via the remote scheduler and pushes on the ROS2 topic `ros2_simple_talker_cpp`.

## Important things to know

* The `ctrlx-datalayer` and `ctrlx-scheduler` Debian packages are required. Please use the [setup-ctrlx-datalayer-deb.sh](../scripts/setup-ctrlx-datalayer-deb.sh) script in the scripts folder to install them.
* An app.automationcore instance must be running on your system. The remote scheduler system will communicate with it and register your callable factory.
* For each callable created by your callable factory, a dedicated thread will be created. This thread will inherit the same settings (priority, affinity, etc.) as the thread in the scheduler.
* Your process will be moved to the cgroup ctrlx.os upon the creation of the first callable, allowing your process to utilize all available cores.
* This snap uses the `process-control` interface. As a high-security interface, it does not auto-connect. If your snap is not signed/granted an exception, you must manually run the following with root access after installation:
&emsp; `sudo snap connect ros2-simple-talker-rs-cpp:process-control`

__Note:__ For simplified sequence tracking, "Console Outputs" are used. In a real-time context, these should be omitted. For tracing capabilities, see the example in the [ctrlX Automation SDK](https://github.com/boschrexroth/ctrlx-automation-sdk/tree/main/samples-cpp/scheduler.remote).

## Prerequisites

* `ros-base` snap. The base snap which provides the ROS 2 runtime binaries. Has to be installed on ctrlX OS. See [ROS 2 Jazzy Base Snap](../ros2-base-jazzy-deb/README.md).
* An Ubuntu based build environment to build an app. See [ctrlX Automation SDK](https://github.com/boschrexroth/ctrlx-automation-sdk).

## Basis for this Project

This project is based on the official ROS 2 Tutorial: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c).

## Building a Snap

Building a snap has two steps:

1. Colcon build: `CMakeLists.txt` defines how compile and link the C++ sources.
2. snap build: `snap/snapcraft.yaml` defines how the compiled binaries are packed into the snap and how they are called on ctrlX OS.

### Colcon Configuration

The colcon build tool is configured by `CMakeLists.txt`.

This section defines the ROS 2 packages needed:

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

And here the executables and their dependencies are defined:

    add_executable(talker src/scheduler_member_function.cpp)
    ament_target_dependencies(talker rclcpp std_msgs)

### Snapcraft Configuration

`snap/snapcraft.yaml` defines how the snap will be build:

* `install/` is dumped into the snap
* also `wrapper/`
* Two apps (talker and listener) are copied into the snap and started as services
* The snap - respectively the executables - uses the content interface of the `ros-base` snap (here the ROS 2 runtime is provided).

### Build the Snap

Start this script:

    ./build-snap-amd64.sh

## About

SPDX-FileCopyrightText: Copyright (c) 2026 Bosch Rexroth AG

<https://www.boschrexroth.com/en/dc/imprint/>

## Licenses

SPDX-License-Identifier: Apache-2.0, MIT
