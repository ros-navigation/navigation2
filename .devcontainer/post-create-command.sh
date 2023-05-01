#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Enable autocomplete for user
cp /etc/skel/.bashrc ~/

# Link Caddyfiles for web apps
ln -s $PWD/.devcontainer/caddy/index.html /srv/index.html
ln -s $PWD/.devcontainer/caddy/index.md /srv/index.md
