#!/bin/bash

SNAP_CONT=ROSDeb
sudo docker run --privileged multiarch/qemu-user-static:latest --reset -p yes
sudo docker build --platform ubuntu/arm64v8  -t arm64v8/ubuntu:jammy -f Docker/runtime.Dockerfile . 
sudo docker run --rm -it --platform linux/arm64 --tmpfs /run --tmpfs /run/lock --cap-add SYS_ADMIN --device=/dev/fuse --security-opt apparmor:unconfined --security-opt seccomp:unconfined -v /sys/fs/cgroup:/sys/fs/cgroup:ro -v /lib/modules:/lib/modules:ro -v $(pwd)/.:/ros_ws/  -d --name=$SNAP_CONT arm64v8/ubuntu:22.04
sudo docker exec -it $SNAP_CONT bash "create_arm_snap.sh"
