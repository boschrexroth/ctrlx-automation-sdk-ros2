#!/bin/bash
set -e

ROOT_DIR=$(git rev-parse --show-toplevel)
source ${ROOT_DIR}/scripts/colcon-build.sh

if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

sudo snapcraft clean
sudo snapcraft --build-for=amd64 --verbosity=verbose