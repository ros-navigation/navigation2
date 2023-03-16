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
