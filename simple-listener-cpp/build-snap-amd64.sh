#!/bin/bash

source colcon-build.sh
if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

snapcraft clean --destructive-mode
snapcraft --destructive-mode