#!/bin/bash

set -exo pipefail

env

cd $OVERLAY_WS
. $UNDERLAY_WS/install/setup.sh

git config --global --add safe.directory "*"
colcon cache lock

colcon build \
    --symlink-install \
    --mixin $OVERLAY_MIXINS

sed --in-place \
    's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
    /ros_entrypoint.sh

# edit apt for autocomplete
mv /etc/apt/apt.conf.d/docker-clean /etc/apt/

# install autocomplete for bash
apt-get update
apt-get install -y \
    bash-completion

# enable autocomplete for user
cp /etc/skel/.bashrc ~/

# source underlay for extentions
echo 'source "$UNDERLAY_WS/install/setup.bash"' >> ~/.bashrc
