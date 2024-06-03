#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Set git config for submodules
git config --local include.path ../.gitconfig

# Set git config for colcon cache
git config --global --add safe.directory "*"

# NOTE: This is slow if not using a mounted volumes, 
# i.e. using workspace from the docker image directly,
# presumably due to docker overlayfs driver overhead.
# If needing to use workspace pre-baked into base image,
# consider using a new volume to be auto populated with
# the workspace pre-baked in image via devcontainer tools.
# Either by deleting old volume from the docker engine
# Or simpler changing volume name in devcontainer.json
sudo chown -R :ubuntu $OVERLAY_WS
# Recursively update group permissions for workspace
# to allow write access via dev users current group
sudo chmod -R g+rwx $OVERLAY_WS

# Recursively update group permissions for ros home
# to allow write access such as ros node logs
sudo chown -R :ubuntu /opt/.ros
sudo chmod -R g+rwx /opt/.ros
