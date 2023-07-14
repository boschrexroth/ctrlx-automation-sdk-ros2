# Simple ROS 2 publisherin C++

ROS 2 node which publishes messages under the topic 'ros2_simple_cpp'.

## Prerequisites

* ros-base-snap: Provides the ROS 2 runtime binaries) has to be installed on ctrlX CORE
* ROS 2 installed on App Build Enviroment

## Basis for this project

[Writing a simple publisher and subscriber (C++)] (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c) was the basis for this project.

## Building a snap

Building a snap has two steps:

1. Colcon build: CMakeLists.txt defines how compile and link the C++ sources.
2. snap build: snap/snapcraft.yaml defines how the compiled binaries are packed into the snap and how they are called on the ctrlX CORE.

### Colcon configuration

The colcon build tool is configured by __CMakeLists.txt__.

This section defines the ROS 2 packages needed:

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

And here the executable and its dependencies is defined:

    add_executable(talker src/publisher_member_function.cpp)
    ament_target_dependencies(talker rclcpp std_msgs)

### Snapcraft configuration

snap/__snapcraft.yaml__ defines how the snap will be build:

* install/ is dumped into the snap
* also wrapper/ 
* An app talker is copied into the snap and started as a service
* The snap - respectively the executable - uses the content interface of the rose-base snap (here the ROS 2 runtime is provided)

### Build the snap

Start this script:

    ./build-snap-amd64.sh

Hint: arm64 build is not supported. Therefor use a bare metal computer.