# ROS2 Humble on ctrlX: A Snap Implementation of the ROS2 Humble Distribution using ROS2 Foxy Debian package for AMD64

## Introduction
We recomment for ROS 2 applications running on a ctrlX CORE this approach:
* Build and install a __base snap__ containing the ROS 2 runtime and the components used for the ctrlX Data Layer access.
* This base snap should provide it's components using the snap content interface.
* Pack your ROS 2 application into one or more overlay snaps.
* Let these overlay snap use the ROS 2 runtime and the ctrlX Data Layer via the content interface of the base snap.
This documentation describes how the base snap can be build.

!!! important
    The build for arm64 devices uses Docker and is in an unreleased state!

## Content of the base snap

The file snap/snapcraft.yaml defines how the base snap will be build.

### Debian packages

The snapcraft plugin 'colcon' is used to download Debian packages:
* Needed during build process: make, gcc, g++
* Needed at runtime: software-properties-common, ros-humble-ros-base, python3-argcomplete, ca-certificates, libzmq5, ctrlx-datalayer

### Python wheels

The snapcraft plugin 'python' is used to download python wheels:
	
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
	
### Content inferface

The base snap makes its files available via the content interface 'executables'.
	
## Building the base snap

### Building base snap on AMD64 for AMD64 targets

Apart from the snap/snapcraft.yaml file and a shell script that calls snapcraft, nothing else is needed here.

Just call:

        ./build-snap.sh

### ARM64 

We recommend to use a bare metal arm64 environment. In this case just call build-snap.sh.

Building an arm64 on an amd64 environment we used Docker.

Call:

        build-snap-arm64-on-amd64.sh

## About

<https://www.boschrexroth.com>

Bosch Rexroth AG  
Bgm.-Dr.-Nebel-Str. 2  
97816 Lohr am Main  
GERMANY  

## Licenses

SPDX-FileCopyrightText: Bosch Rexroth AG
SPDX-License-Identifier: MIT
