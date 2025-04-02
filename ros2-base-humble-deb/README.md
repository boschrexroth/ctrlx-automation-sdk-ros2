# ROS 2 Humble on ctrlX OS: A Snap Implementation of the ROS 2 Humble Distribution using ROS 2 Debian Package

## Introduction

We recommend for ROS 2 applications running on a ctrlX OS this approach:

* Build and install a __base snap__ containing the ROS 2 runtime and the components used for the ctrlX Data Layer access.
* This base snap provides its components using the snap [content interface](https://snapcraft.io/docs/content-interface).
* Pack your ROS 2 application into one or more overlay snaps.
* Let these overlay snap use the ROS 2 runtime and the ctrlX Data Layer via the content interface of the __base snap__.

This documentation describes how the base snap `ros2-base` can be build.

!!! important
    The build for arm64 devices uses Docker and is in an unreleased state!

## Content of the Base Snap

The file `snap/snapcraft.yaml` defines how the base snap will be build.

### Debian Packages

The snapcraft plugin [colcon](https://snapcraft.io/docs/colcon-plugin) is used to download Debian packages:

* Needed during build process: make, gcc, g++
* Needed at runtime: software-properties-common, ros-humble-ros-base, python3-argcomplete, ca-certificates, ctrlx-datalayer

### Python Wheels

The snapcraft plugin [python](https://snapcraft.io/docs/python-plugin) is used to download python wheels:

* python3-wheel
* ctrlx-datalayer
* empy
* numpy
* rosdep
* rosdistro
* colcon-core
* lark rosdep
* rosdistro
* colcon-core
* lark

### Content Interface

The base snap makes its files available via the content interface `executables`.

## Building the Base Snap

### AMD64

Compiling for AMD64 on an AMD64 machine is straight-forward. Apart from the `snap/snapcraft.yaml` file and a shell script that calls snapcraft, nothing else is needed here.

Call:

        ./build-snap-amd64.sh

### ARM64

We recommend using a native ARM64 host environment to compile the ARM64 code. Set up a corresponding hardware, for example __Raspberry Pi__ or a virtual environment, for example __Qemu__ [installation scripts](https://github.com/boschrexroth/ctrlx-automation-sdk/tree/main/scripts/environment).

## About

SPDX-FileCopyrightText: Copyright (c) 2023-2025 Bosch Rexroth AG

<https://www.boschrexroth.com/en/dc/imprint/>

## Licenses

SPDX-License-Identifier: MIT
