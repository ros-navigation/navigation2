#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

cd $OVERLAY_WS

colcon cache lock

BUILD_UNFINISHED=$(
    colcon list \
    --names-only \
    --packages-skip-build-finished \
    | xargs)
echo BUILD_UNFINISHED: $BUILD_UNFINISHED

BUILD_FAILED=$(
    colcon list \
    --names-only \
    --packages-select-build-failed \
    | xargs)
echo BUILD_FAILED: $BUILD_FAILED

BUILD_INVALID=$(
    colcon list \
    --names-only \
    --packages-select-cache-invalid \
    --packages-select-cache-key build \
    | xargs)
echo BUILD_INVALID: $BUILD_INVALID

BUILD_PACKAGES=""
if [ -n "$BUILD_UNFINISHED" ] || \
    [ -n "$BUILD_FAILED" ] || \
    [ -n "$BUILD_INVALID" ]
then
    BUILD_PACKAGES=$(
    colcon list \
        --names-only \
        --packages-above \
            $BUILD_UNFINISHED \
            $BUILD_FAILED \
            $BUILD_INVALID \
    | xargs)
fi
echo BUILD_PACKAGES: $BUILD_PACKAGES

# DEBUG: Uncoment for more sterile but slower builds
# colcon clean packages --yes \
#     --packages-select ${BUILD_PACKAGES} \
#     --base-select build install

# OPTOINAL: Uncomment to build packages upon update
. /opt/ros/$ROS_DISTRO/setup.sh
colcon build \
    --mixin $OVERLAY_MIXINS \
    --packages-select ${BUILD_PACKAGES} \
    || true
