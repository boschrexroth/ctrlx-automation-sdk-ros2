# Scrips Description

## Installation Scripts Files

* __setup-system-jazzy-noble.sh__ installs the ROS2 Jazzy development environment [Install Ubuntu (deb packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
* __setup-ctrlx-datalayer-deb.sh__ installs the ctrlx-datalayer Debian package and registers its storage directory as local package source
* __install-ctrlx-datalayer-deb.sh__ installs the ctrlx-datalayer Debian package, see [ctrlX AUTOMATION SDK/Releases](https://github.com/boschrexroth/ctrlx-automation-sdk/releases)

``` bash
# install default debian package
./setup-ctrlx-datalayer-deb.sh 
```

``` bash
# install from SDK version 4.4.0 debian package version 3.4.1
./install-ctrlx-datalayer-deb.sh 4.4.0 3.4.1
```

## Build Scripts Files

* __colcon-build.sh__ helper script for colon build, base on ROS2 __Jazzy__
* __build-snap-cpp.sh__ helper script for creating snaps based on c++ projects
* __build-snap-py.sh__ Help script for creating snaps based on python projects

SPDX-License-Identifier: MIT
