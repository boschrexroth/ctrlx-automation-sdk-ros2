#!/bin/bash

rosdep install -i --from-path src --rosdistro jazzy -y
if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

source /opt/ros/jazzy/setup.bash
colcon build
