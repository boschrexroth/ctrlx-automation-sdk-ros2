# ctrlX AUTOMATION Software Development Kit for ROS 2

This is the Software Development Kit (SDK) to build ROS 2 Applications, that can run on industrial devices that are based on the ctrlX OS platform. 

The **Robot Operating System (ROS)** is an open source robotics software framework that help you build robot applications. 
**ctrlX OS** is an industrial grade realtime platform based on Linux which is available for several industrial hardware devices and features an app-based modular architecture that allows you to install additional functionality. This includes apps for industrial fieldbus communication, machine automation, programmable logic control (PLC) and other automation software which can also be found ready-to-run in the [ctrlX Store](https://developer.community.boschrexroth.com/).

## Getting Started

As a prerequisite you should get yourself familiar with:

* ROS 2 and its underlying architecture using the official ROS 2 documentation at: <https://www.ros.org/blog/getting-started/>
* The [ctrlX AUTOMATION Ecosystem](https://ctrlx-automation.com/) and the architecture of ctrlX OS as well as devices which are capable to run ctrlX OS. E.g. ctrlX CORE devices from [Bosch Rexroth](https://www.boschrexroth.com/).
* The [ctrlX AUTOMATION SDK](https://github.com/boschrexroth/ctrlx-automation-sdk) which is the underlying SDK that allows you to create any kind of App for ctrlX OS and is the foundation for the ctrlX AUTOMATION Software Development Kit for ROS 2.

## Usage

This SDK builds on top of the [ctrlX AUTOMATION SDK](https://github.com/boschrexroth/ctrlx-automation-sdk) to make apps for ctrlX OS, which is itself based on Ubuntu Core Distribution and a Linux kernel with PREEMPT_RT patch.

ctrlX OS, Ubuntu Core, as well as ROS 2 are released as *distributions*. This release of the SDK is intended to be used with:

* ctrlX OS 2.x (Includes Ubuntu Core 22)
* ROS 2 [Humble Hawksbill](https://docs.ros.org/en/humble/)

All projects use [Visual Studio Code](https://code.visualstudio.com/) as editor.

### ROS 2 Humble Base Snap

To build and deploy a ROS 2 application to ctrlX OS it needs to be package as [snap](https://ubuntu.com/core/services/guide/snaps-intro). In order to make your development process as efficient as possible we recommend splitting your ROS 2 deployment into at least two snaps. A so-called *base snap* is used to encapsulate the ROS 2 runtime as well as libraries which are dependencies to your application. The *base snap* is installed on ctrlX OS once and provides the compiled runtime and libraries to one or more ROS 2 application snaps which hold your business logic. Following this approach you can reduce build times and resource footprint on the device by sharing the ROS 2 dependencies.

For details have a look at the instructions for the [ROS 2 Humble Base Snap](ros2-base-humble-deb/README.md) which also shows you how to compile ROS 2 for ctrlX OS.

If you nevertheless want to go with a single app, then use the *base snap* and integrate your ROS 2 application directly.

## Samples

This SDK contains also multiple project examples to show the usage of ROS 2. All examples build on top of the *base snap*.

### ROS 2 Humble Applications in C++

#### Writing a Simple Publisher and Subscriber (C++)

This example is based on the official ROS 2 Tutorial and adjusted to run on ctrlX OS together with the *base snap*. You can find the original source code at [ROS 2 Humble Tutorials - Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c)

* [`simple-talker-cpp/`](simple-talker-cpp/README.md) contains the snap version of the described publisher (talker)
* [`simple-listener-cpp/`](simple-listener-cpp/README.md) contains the snap version of the described subscriber (listener)

### ROS 2 Humble Applications in Python

#### Writing a Simple Publisher and Subscriber (Python)

This example is based on the official ROS 2 Tutorial and adjusted to run on ctrlX OS together with the *base snap*. You can find the original source code at [ROS 2 Humble Tutorials - Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

* [`simple-talker-py/`](simple-talker-py/README.md) contains the snap version of the described publisher (talker)
* [`simple-listener-py/`](simple-listener-py/README.md) contains the snap version of the described subscriber (listener)

#### Writing a Simple Publisher and Subscriber with ctrlX Data Layer Access (Python)

Both sample projects are based on the same official ROS 2 tutorial as mentioned above. Additionally, the example is extended with access to the ctrlX Data Layer using the ctrlX SDK.

* [`simple-talker-dl-py/`](simple-talker-dl-py/README.md) Reads a Data Layer value and publishes it as ROS 2 message under a specific topic.
* [`simple-listener-dl-py/`](simple-listener-dl-py/README.md) Subscribes messages of the topic, receives the value and writes it into the ctrlX Data Layer.

## Support

This repository is provided and maintained by [Bosch Rexroth](https://www.boschrexroth.com). Feel free to check out and be part of the [ctrlX AUTOMATION Community](https://ctrlx-automation.com/community). Get additional support, e.g. related to Bosch Rexroth Devices, Apps, SDKs and Services, or leave some ideas and feedback.

To report bugs, request changes and discuss new ideas you may also have a look at the issue tracker of this repository:
<https://github.com/boschrexroth/ctrlx-automation-sdk-ros2/issues>

## Important directions for use

### Areas of use and application

The content (e.g. source code and related documents) of this repository is intended to be used for configuration, parameterization, programming or diagnostics in combination with selected Bosch Rexroth ctrlX AUTOMATION devices.
Additionally, the specifications given in the "Areas of Use and Application" for ctrlX AUTOMATION devices used with the content of this repository do also apply.

### Unintended use

Any use of the source code and related documents of this repository in applications other than those specified above or under operating conditions other than those described in the documentation and the technical specifications is considered as "unintended". Furthermore, this software must not be used in any application areas not expressly approved by Bosch Rexroth.

## Changelog

```text
* 2023-09-01: 1.0.0 - Initial release for ROS 2 Humble Hawksbill.
```

## About

SPDX-FileCopyrightText: Copyright (c) 2023 Bosch Rexroth AG

https://www.boschrexroth.com/en/dc/imprint/

Bosch Rexroth AG  
Bgm.-Dr.-Nebel-Str. 2  
97816 Lohr am Main  
GERMANY

## Licenses


SPDX-License-Identifier: MIT
