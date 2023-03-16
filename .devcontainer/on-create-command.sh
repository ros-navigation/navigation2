#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

cd $OVERLAY_WS
. $UNDERLAY_WS/install/setup.sh

git config --global --add safe.directory "*"
colcon cache lock

colcon build \
    --symlink-install \
    --mixin $OVERLAY_MIXINS
