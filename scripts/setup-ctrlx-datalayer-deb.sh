#!/bin/bash

# Downloads the ctrlX Data Layer Debian packages (via install-ctrlx-datalayer-deb.sh,
# optional $1 = SDK release version), then registers the local "deb" folder as a
# trusted APT repository and installs the ctrlx-datalayer and ctrlx-scheduler
# packages so that sample project snaps can be built against them.

# Exit immediately if a command exits with a non-zero status.
#set -e

ROOT_DIR=$(git rev-parse --show-toplevel)

echo "rootdir=$ROOT_DIR"

rm -rf "${ROOT_DIR}"/deb

# shellcheck source=/dev/null
source "${ROOT_DIR}"/scripts/install-ctrlx-datalayer-deb.sh "${1}"
pushd "${ROOT_DIR}"/deb || return

  sudo apt-get install -y dpkg-dev

# Install debian package locally so that 'apt-get install' will find it (for building sample project snaps)
  dpkg-scanpackages -m . >Packages

# Add package to sources list
  full_path=$(pwd)
  echo "deb [trusted=yes] file:${full_path} ./" | sudo tee /etc/apt/sources.list.d/ctrlx-automation.list

# Fix the APT Error Download that is performed unsandboxed as root as
  echo "APT::Sandbox::User \"root\";" | sudo tee /etc/apt/apt.conf.d/10sandbox

  sudo apt-get update

# Install newest ctrlx-datalayer package
  sudo apt-get install ctrlx-datalayer ctrlx-scheduler -y
popd || return
