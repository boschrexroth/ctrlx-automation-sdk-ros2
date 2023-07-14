#!/bin/bash

# https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

# Uninstall the Docker Engine, CLI, containerd, and Docker Compose packages --------------------------------
sudo apt-get purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin docker-ce-rootless-extras

# To delete all images, containers, and volumes -----------------
sudo rm -rf /var/lib/docker
sudo rm -rf /var/lib/containerd
