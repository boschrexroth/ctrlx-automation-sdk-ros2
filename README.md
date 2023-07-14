![ROS 2 Humble](https://docs.ros.org/en/humble/_static/humble-small.png)

## Find out the supported samples

### ROS 2 Humble Base Snap

* [ROS 2 Humble Base Snap](ros2-base-humble-deb/README.md)

### ROS 2 Humble Applications in Python

#### Writing a simple publisher and subscriber (Python)

The templates for these project you can find under [ROS 2 Humble Tutorials - Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

* simple-talker-py/ contains the snap version of the described publisher (talker)

* simple-listener-py/ contains the snap version of the described subscriber (listener)

#### Writing a simple publisher and subscriber with ctrlX Data Layer access (Python)

Both sample projects are based on the same tutorial above mentioned. Additionally, the access nodes (data points) of the ctrlX Data Layer.

* simple-talker-dl-py/ Reads a Data Layer value and publishes it as ROS 2 message under a specific topic.

* simple-listener-dl-py/ Subscribes messages of the topic, receives the value and writes it into the ctrlX Data Layer.

### ROS 2 Humble Applications in C++

#### Writing a simple publisher and subscriber (Python)

The templates for these project you can find under [ROS 2 Humble Tutorials - Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c)

* simple-talker-cpp/ contains the snap version of the described publisher (talker)

* simple-listener-cpp/ contains the snap version of the described subscriber (listener)

## Licenses

SPDX-FileCopyrightText: Bosch Rexroth AG
SPDX-License-Identifier: MIT
