#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Enable autocomplete for user
cp /etc/skel/.bashrc ~/

# Enable autocomplete using colcon
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Add aliases to .bashrc
echo "alias sows='source $OVERLAY_WS/install/setup.bash'" >> ~/.bashrc
