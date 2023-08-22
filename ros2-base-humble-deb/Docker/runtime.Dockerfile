ARG BASE_IMAGE=arm64v8/ubuntu:22.04
FROM $BASE_IMAGE

WORKDIR /ros_ws

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y fuse snapd snap-confine squashfuse sudo init
RUN apt-get clean
RUN dpkg-divert --local --rename --add /sbin/udevadm 
RUN ln -s /bin/true /sbin/udevadm
RUN systemctl enable snapd

VOLUME ["/sys/fs/cgroup"]
STOPSIGNAL SIGRTMIN+3
ENTRYPOINT ["/sbin/init"]
SHELL ["/bin/bash", "-c"] 
