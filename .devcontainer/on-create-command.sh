#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

cd $OVERLAY_WS

git config --global --add safe.directory "*"
colcon cache lock

. $UNDERLAY_WS/install/setup.sh
colcon build \
    --symlink-install \
    --mixin $OVERLAY_MIXINS
