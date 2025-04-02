#!/bin/bash

rosdep install -i --from-path src --rosdistro humble -y
if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

source /opt/ros/humble/setup.bash
colcon build