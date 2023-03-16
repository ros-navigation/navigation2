#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Enable autocomplete for user
cp /etc/skel/.bashrc ~/
