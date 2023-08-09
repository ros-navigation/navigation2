#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

git config --global --add safe.directory "*"

.devcontainer/update-content-command.sh
