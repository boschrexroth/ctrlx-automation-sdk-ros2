#!/bin/bash

# Downloads the ctrlX Automation SDK release archive (version = $1, default 4.6.0)
# from GitHub and extracts its Debian packages into <repo-root>/deb.

#set -e
SDK_RELEASE_VERSION=${1:-4.6.0}
TMP_DEBS="/tmp/deb"
GITHUB_TOKEN_LOCAL=${GITHUB_TOKEN:-}

ROOT_DIR=$(git rev-parse --show-toplevel)

rm -rf "${ROOT_DIR}"/deb
mkdir -p "${ROOT_DIR}"/deb

rm -rf "${TMP_DEBS}"
mkdir -p "${TMP_DEBS}"

if [ -n "${GITHUB_TOKEN_LOCAL}" ]; then
  wget --progress=bar:force --header="Authorization: Bearer ${GITHUB_TOKEN_LOCAL}" https://github.com/boschrexroth/ctrlx-automation-sdk/releases/download/"${SDK_RELEASE_VERSION}"/ctrlx-automation-sdk-"${SDK_RELEASE_VERSION}".zip -P "${TMP_DEBS}/"
else
  wget --progress=bar:force https://github.com/boschrexroth/ctrlx-automation-sdk/releases/download/"${SDK_RELEASE_VERSION}"/ctrlx-automation-sdk-"${SDK_RELEASE_VERSION}".zip -P "${TMP_DEBS}/"
fi
unzip "${TMP_DEBS}/ctrlx-automation-sdk-${SDK_RELEASE_VERSION}.zip" "ctrlx-automation-sdk/deb/*" -d "${ROOT_DIR}"/deb/
