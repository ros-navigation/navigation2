#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Use first argument as target name
target=$1

# Bake the target and export locally to static tag
docker buildx bake --load \
    --file docker-bake.hcl \
    --set $target.tags=nav2:devcontainer \
    $target
