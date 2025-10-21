#!/bin/bash

#set -e
SDK_RELEASE_VERSION=${1:-3.4.0}
DATALAYER_DEB_FILE_VERSION=${2:-2.8.6}

ROOT_DIR=$(git rev-parse --show-toplevel)

rm -rf ${ROOT_DIR}/deb
mkdir -p ${ROOT_DIR}/deb

wget --quiet https://github.com/boschrexroth/ctrlx-automation-sdk/releases/download/${SDK_RELEASE_VERSION}/ctrlx-datalayer-${DATALAYER_DEB_FILE_VERSION}.deb -P ${ROOT_DIR}/deb
