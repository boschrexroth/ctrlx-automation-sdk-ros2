#!/bin/bash

#set -e
SDK_RELEASE_VERSION=${1:-4.6.0}
TMP_DEBS="/tmp/deb"

ROOT_DIR=$(git rev-parse --show-toplevel)

rm -rf "${ROOT_DIR}"/deb
mkdir -p "${ROOT_DIR}"/deb

rm -rf "${TMP_DEBS}"
mkdir -p "${TMP_DEBS}"

wget --quiet https://github.com/boschrexroth/ctrlx-automation-sdk/releases/download/"${SDK_RELEASE_VERSION}"/ctrlx-automation-sdk-"${SDK_RELEASE_VERSION}".zip -P "${TMP_DEBS}/"
unzip "${TMP_DEBS}/ctrlx-automation-sdk-${SDK_RELEASE_VERSION}.zip" "ctrlx-automation-sdk/deb/*" -d "${ROOT_DIR}"/deb/
