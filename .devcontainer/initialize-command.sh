#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Use first argument as target name
target=$1

################################################################################
# MARK: Pull image - download image from CI and GHCR for local dev container
# REFERENCE_IMAGE=ghcr.io/ros-navigation/navigation2:main-debugger
# docker pull $REFERENCE_IMAGE
# export DEV_FROM_STAGE=$REFERENCE_IMAGE
################################################################################

# Bake the target and export locally to static tag
docker buildx bake --load \
    --file docker-bake.hcl \
    --set $target.tags=nav2:devcontainer \
    $target

# mkdir -p $HOME/nav2
