#!/bin/bash

set -exo pipefail

export OVERLAY_MIXINS="release ccache lld"
export CCACHE_DIR="$OVERLAY_WS/.ccache"
env

cd $OVERLAY_WS
. $UNDERLAY_WS/install/setup.sh

colcon cache lock

colcon build \
    --symlink-install \
    --mixin $OVERLAY_MIXINS

sed --in-place \
    's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
    /ros_entrypoint.sh
