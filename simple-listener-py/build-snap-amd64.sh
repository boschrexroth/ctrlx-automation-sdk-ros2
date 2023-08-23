#!/bin/bash
colcon build
snapcraft clean --destructive-mode
snapcraft --destructive-mode