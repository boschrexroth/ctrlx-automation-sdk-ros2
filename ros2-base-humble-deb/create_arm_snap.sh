#!/bin/bash
#
# Script to run inside a docker runtime container
# For snapcraft core* installation it follows the following description: 
# https://raw.githubusercontent.com/snapcore/snapcraft/master/docker/Dockerfile

mkdir -p snap

apt-get update
apt-get install -y curl jq squashfs-tools
apt-get install -y git
apt-get install -y make 
apt-get install -y build-essential
apt-get install -y zlib1g-dev
apt-get install -y liblzma-dev
apt-get install -y liblzo2-dev
apt-get install -y liblz4-dev
apt-get install -y ca-certificates
git clone https://github.com/plougher/squashfs-tools.git
cd squashfs-tools && \
	git checkout 4.5.1 && \
	sed -Ei 's/#(XZ_SUPPORT.*)/\1/' squashfs-tools/Makefile && \
	sed -Ei 's/#(LZO_SUPPORT.*)/\1/' squashfs-tools/Makefile && \
	sed -Ei 's/#(LZ4_SUPPORT.*)/\1/' squashfs-tools/Makefile && \
	sed -Ei 's|(INSTALL_PREFIX = ).*|\1 /usr|' squashfs-tools/Makefile && \
	sed -Ei 's/\$\(INSTALL_DIR\)/$(DESTDIR)$(INSTALL_DIR)/g' squashfs-tools/Makefile && \
	cd squashfs-tools && \
	make -j$(nproc) && \
	make install

# Download and unpack snapd
mkdir -p /snap/snapd/current
snap download snapd
unsquashfs -f -d /snap/snapd/current snapd_*.snap

# Replace mksquashfs and unsqusahfs with our own As reported here: https://bugs.launchpad.net/snapd/+bug/1916552
cp /usr/bin/mksquashfs /snap/snapd/current/usr/bin
cp /usr/bin/unsquashfs /snap/snapd/current/usr/bin

# Repack snapd
mksquashfs /snap/snapd/current /snapd.snap
cd ../..
pwd
# Grab the core snap (for backwards compatibility) from the stable channel and
# unpack it in the proper place.
if [ ! -d "/snap/core" ]; then
    curl -L -H 'Snap-CDN: none' $(curl -kH 'X-Ubuntu-Series: 16' -H "X-Ubuntu-Architecture: arm64" 'https://api.snapcraft.io/api/v1/snaps/details/core' | jq '.download_url' -r) --output core.snap
    mkdir -p /snap/core
    unsquashfs -d /snap/core/x1 core.snap
    ln -s x1 /snap/core/current
fi

# Grab the core18 snap (which snapcraft uses as a base) from the stable channel
# and unpack it in the proper place.
if [ ! -d "/snap/core18" ]; then
    curl -L -H 'Snap-CDN: none' $(curl -kH 'X-Ubuntu-Series: 16' -H "X-Ubuntu-Architecture: arm64" 'https://api.snapcraft.io/api/v1/snaps/details/core18' | jq '.download_url' -r) --output core18.snap
    mkdir -p /snap/core18
    unsquashfs -d /snap/core18/x1 core18.snap
    ln -s x1 /snap/core18/current
fi

# Grab the core20 snap from the stable channel and
# unpack it in the proper place.
if [ ! -d "/snap/core20" ]; then
    curl -L -H 'Snap-CDN: none' $(curl -kH 'X-Ubuntu-Series: 16' -H "X-Ubuntu-Architecture: arm64" 'https://api.snapcraft.io/api/v1/snaps/details/core20' | jq '.download_url' -r) --output core20.snap
    mkdir -p /snap/core20
    unsquashfs -d /snap/core20/x1 core20.snap
    ln -s x1 /snap/core20/current
fi

# Grab the core22 snap from the stable channel and
# unpack it in the proper place.
if [ ! -d "/snap/core22" ]; then
    curl -L -H 'Snap-CDN: none' $(curl -kH 'X-Ubuntu-Series: 16' -H "X-Ubuntu-Architecture: arm64" 'https://api.snapcraft.io/api/v1/snaps/details/core22' | jq '.download_url' -r) --output core20.snap
    mkdir -p /snap/core22
    unsquashfs -d /snap/core22/x1 core22.snap
    ln -s x1 /snap/core22/current
fi

# Grab the snapcraft snap from the $SNAPCRAFT channel and unpack it in the proper
# place.
if [ ! -d "/snap/snapcraft" ]; then
    curl -L -H 'Snap-CDN: none' $(curl -kH 'X-Ubuntu-Series: 16' -H "X-Ubuntu-Architecture: arm64" 'https://api.snapcraft.io/api/v1/snaps/details/snapcraft?channel='$SNAPCRAFT | jq '.download_url' -r) --output snapcraft.snap
    mkdir -p /snap/snapcraft
    unsquashfs -d /snap/snapcraft/x1 snapcraft.snap
    ln -s x1 /snap/snapcraft/current
fi

# Fix Python3 Installation: 
unlink /snap/snapcraft/current/usr/bin/python3
ln -s /snap/snapcraft/current/usr/bin/python3.* /snap/snapcraft/current/usr/bin/python3
echo /snap/snapcraft/current/lib/python3.*/site-packages >> /snap/snapcraft/current/usr/lib/python3/dist-packages/site-packages.pth

# Create a snapcraft runner (TODO: move version detection to the core of
# snapcraft).
mkdir -p /snap/bin
echo '#!/bin/sh' > /snap/bin/snapcraft
snap_version="$(awk '/^version:/{print $2}' /snap/snapcraft/current/meta/snap.yaml)" && echo "export SNAP_VERSION=\"$snap_version\"" >> /snap/bin/snapcraft
echo 'export SNAP="/snap/snapcraft/current"' >> /snap/bin/snapcraft
echo 'export SNAP_NAME="snapcraft"' >> /snap/bin/snapcraft
echo "export SNAP_ARCH=\"arm64\"" >> /snap/bin/snapcraft
echo 'exec "$SNAP/usr/bin/python3" "$SNAP/bin/snapcraft" "$@"' >> /snap/bin/snapcraft
chmod +x /snap/bin/snapcraft

export PATH=/snap/bin:$PATH
export SNAPCRAFT_BUILD_ENVIRONMENT="host"
echo SNAPCRAFT_BUILD_ENVIRONMENT=$SNAPCRAFT_BUILD_ENVIRONMENT >> /etc/environment
export SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH=1
echo SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH=$SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH >> /etc/environment
export SNAPCRAFT_BUILD_INFO=1
export PYTHONPATH=/snap/snapcraft/current/lib/python3.10/site-packages/:$PYTHONPATH

# Install ROS2 in order for colcon plugin to be used to be used. https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
apt update &&  apt-get install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


apt-get install -y software-properties-common
add-apt-repository universe
apt-get update 

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt-get install -y ca-certificates
apt-get install -y ros-humble-ros-base python3-argcomplete
apt-get install -y ros-dev-tools
apt-get install -y python3-rosdep
rosdep init

snapcraft clean --destructive-mode
snapcraft  --destructive-mode --target-arch=arm64 

