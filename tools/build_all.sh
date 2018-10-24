#!/bin/bash

# All the parenthesis around the calls to setup.bash and the build are to
# prevent the ROS variables from entering the global scope of this script.
# It seems that the ros1_bridge build will fail if the ros2 environment is
# sourced before ROS 1, so we need to make sure that no ROS 2 variables are
# set at the time we start the last step, the ros1_bridge build.

if [ "$ROS1_DISTRO" = "" ]; then
	export ROS1_DISTRO=kinetic
fi
if [ "$ROS2_DISTRO" = "" ]; then
	export ROS2_DISTRO=bouncy
fi
if test "$ROS1_DISTRO" != "kinetic" && test "$ROS1_DISTRO" != "melodic" ; then
	echo "ROS1_DISTRO variable must be set to either kinetic or melodic"
	exit 1
fi
if [ "$ROS2_DISTRO" != "bouncy" ]; then
	echo "ROS2_DISTRO variable must be set to bouncy"
	exit 1
fi

set -e

CWD=`pwd`

# Determine which repos to build. If the ROS 1 and ROS 2 base directories are
# not present, then don't build
if [ -d "ros1_dependencies_ws" ]; then
  ENABLE_ROS1=true
else
  ENABLE_ROS1=false
fi
if [ -d "ros2_ws" ]; then
  ENABLE_ROS2=true
  ROS2_SETUP_FILE=$CWD/ros2_ws/install/setup.bash
else
  ENABLE_ROS2=false
  ROS2_SETUP_FILE=/opt/ros/$ROS2_DISTRO/setup.bash
fi


# Build ROS 1 dependencies
if [ "$ENABLE_ROS1" = true ]; then
  cd ros1_dependencies_ws
  ROS1_SETUP_FILE=/opt/ros/$ROS1_DISTRO/setup.bash 
  if [ ! -f $ROS1_SETUP_FILE ]; then
    echo "'$ROS1_SETUP_FILE' does not exist. Install ROS $ROS1_DISTRO"
    exit 1
  fi 
  (. $ROS1_SETUP_FILE && catkin_make)
 fi

# Build ROS 2 base
if [ "$ENABLE_ROS2" = true ]; then
  cd $CWD/ros2_ws
  colcon build --symlink-install --packages-skip ros1_bridge
fi

# Build our ROS 2 dependencies
cd $CWD/navstack_dependencies_ws
(. $ROS2_SETUP_FILE &&
 colcon build --symlink-install)

# Build our code
cd $CWD/navigation2_ws
(. $ROS2_SETUP_FILE && . $CWD/navstack_dependencies_ws/install/setup.bash &&
 colcon build --symlink-install)

# Update the ROS1 bridge
if test "$ENABLE_ROS1" = true && test "$ENABLE_ROS2" = true ; then
  cd $CWD
  . ros1_dependencies_ws/devel/setup.bash
  . navigation2_ws/install/setup.bash
  cd $CWD/ros2_ws
  colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
fi
