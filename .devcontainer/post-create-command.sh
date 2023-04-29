#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Enable autocomplete for user
cp /etc/skel/.bashrc ~/

# Link Caddyfiles for web apps
ln -s $PWD/.devcontainer/foxglove/Caddyfile /opt/foxglove/Caddyfile
