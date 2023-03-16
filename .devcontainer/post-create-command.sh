#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# enable autocomplete for user
cp /etc/skel/.bashrc ~/
