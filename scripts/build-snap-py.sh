#!/bin/bash
set -e

ROOT_DIR=$(git rev-parse --show-toplevel)
source "${ROOT_DIR}"/scripts/colcon-build.sh

if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

snapcraft clean
snapcraft pack --build-for=amd64 --verbosity=verbose
