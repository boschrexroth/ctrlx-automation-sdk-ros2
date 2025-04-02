# Scrips Description

## Installation Scripts Files

* __setup-ctrlx-datalayer-deb.sh__ installs the ctrlx-datalayer Debian package and registers its storage directory as local package source
* __install-ctrlx-datalayer-deb.sh__ installs the ctrlx-datalayer Debian package, see [ctrlX AUTOMATION SDK/Releases](https://github.com/boschrexroth/ctrlx-automation-sdk/releases)
* __setup-system-humble-jammy.sh__ installs the ROS2 Humble development environment [Install Ubuntu (deb packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

``` bash
# install default debian package
./install-ctrlx-datalayer-deb.sh 
```

``` bash
# install from SDK version 3.2.0 debian package version 2.7.5
./install-ctrlx-datalayer-deb.sh 3.2.0 2.7.5
```

## Build Scripts Files

* __colcon-build.sh__ helper script for colon build, base on ROS2 __Humble__
* __build-snap-cpp.sh__ helper script for creating snaps based on c++ projects
* __build-snap-py.sh__ Help script for creating snaps based on python projects

SPDX-License-Identifier: MIT
