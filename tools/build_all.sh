#!/bin/bash

# All the parenthesis around the calls to setup.bash and the build are to
# prevent the ROS variables from entering the global scope of this script.
# It seems that the ros1_bridge build will fail if the ros2 environment is
# sourced before ROS 1, so we need to make sure that no ROS 2 variables are
# set at the time we start the last step, the ros1_bridge build.

if [ "$ROS2_DISTRO" = "" ]; then
  export ROS2_DISTRO=foxy
fi
if [ "$ROS2_DISTRO" != "foxy" ]; then
  echo "ROS2_DISTRO variable must be set to foxy"
  exit 1
fi

set -e

# so you can call from anywhere in the nav2_ws, ros2_ws, or deps branches
while [[ "$PWD" =~ ros2_ws ]]; do
  cd ../
done

while [[ "$PWD" =~ ros2_nav_dependencies_ws ]]; do
  cd ../
done

while [[ "$PWD" =~ nav2_ws ]]; do
  cd ../
done

CWD=`pwd`

# Determine which repos to build. If ROS 2 base directories are
# not present, then don't build
if [ -d "ros2_ws" ]; then
  ENABLE_ROS2=true
  ROS2_SETUP_FILE=$CWD/ros2_ws/install/setup.bash
else
  ENABLE_ROS2=false
  ROS2_SETUP_FILE=/opt/ros/$ROS2_DISTRO/setup.bash
fi

# Build ROS 2 base
if [ "$ENABLE_ROS2" = true ]; then
  cd $CWD/ros2_ws
  colcon build --symlink-install --packages-skip ros1_bridge
fi

# Build our ROS 2 dependencies
cd $CWD/ros2_nav_dependencies_ws
export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
(. $ROS2_SETUP_FILE &&
 colcon build --symlink-install)

# Build our code
cd $CWD/nav2_ws
export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
(. $ROS2_SETUP_FILE && . $CWD/ros2_nav_dependencies_ws/install/setup.bash &&
 colcon build --symlink-install)

# Update the ROS1 bridge
if test "$ENABLE_ROS1" = true && test "$ENABLE_ROS2" = true ; then
  cd $CWD
  . nav2_ws/install/setup.bash
  cd $CWD/ros2_ws
  colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
fi
