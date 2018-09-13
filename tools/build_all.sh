#!/bin/bash

# All the parenthesis around the calls to setup.bash and the build are to
# prevent the ROS variables from entering the global scope of this script.
# It seems that the ros1_bridge build will fail if the ros2 environment is
# sourced before ROS 1, so we need to make sure that no ROS 2 variables are
# set at the time we start the last step, the ros1_bridge build.

set -e

CWD=`pwd`

# Build ROS 1 dependencies
cd ros1_dependencies_ws
(source /opt/ros/kinetic/setup.bash &&
 catkin_make)

# Build ROS 2 base
cd $CWD/ros2_ws
colcon build --symlink-install --packages-skip ros1_bridge

# Build our ROS 2 dependencies
cd $CWD/navstack_dependencies_ws
(source $CWD/ros2_ws/install/setup.bash &&
 colcon build --symlink-install)

# Build our code
cd $CWD/navigation2
(source $CWD/navstack_dependencies_ws/install/setup.bash &&
 colcon build --symlink-install)

# Update the ROS1 bridge
cd $CWD
source ros1_dependencies_ws/devel/setup.bash
source navigation2/install/setup.bash
cd $CWD/ros2_ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
