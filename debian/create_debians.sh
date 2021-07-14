#!/bin/bash

apt update && apt install -y python3-pip fakeroot dpkg-dev debhelper dh-python && pip3 install bloom

# store the current dir
CUR_DIR=$(pwd)

# Update ROS deps
rosdep update &> /dev/null

# Package list to create and install debian. 
# Watch out the order to avoid missing dependencies while creating the debian.
PACKAGE_LIST_UNDERLAY=(
            BehaviorTree.CPP \
            angles/angles
)

for PACKAGE in ${PACKAGE_LIST_UNDERLAY[@]}; do
    echo ""
    echo "Creating debian for $PACKAGE..."

    # We have to go to the ROS package parent directory
    cd /opt/underlay_ws/src/$PACKAGE;
    bloom-generate rosdebian --ros-distro foxy &> /dev/null
    fakeroot debian/rules "binary --parallel" &> /dev/null

    cd ..
    DEB_FILE=$(find *.deb);
    dpkg -i $DEB_FILE
    rm *.deb *.ddeb
    cd $CUR_DIR

done

# Package list to create and install debian. 
# Watch out the order to avoid missing dependencies while creating the debian.
PACKAGE_LIST=(
            navigation2/geographic_msgs \
            navigation2/nav2_msgs \
            navigation2/nav2_common \
            navigation2/nav2_util \
            navigation2/nav2_voxel_grid
            navigation2/nav2_behavior_tree \
            navigation2/nav2_bt_navigator \
            navigation2/nav2_map_server \
            navigation2/nav2_lifecycle_manager \
            navigation2/nav2_costmap_2d \
            navigation2/nav2_core \
            navigation2/nav2_rviz_plugins \
            navigation2/nav2_amcl \
            navigation2/nav2_planner \
            navigation2/nav2_navfn_planner
)

for PACKAGE in ${PACKAGE_LIST[@]}; do
    echo ""
    echo "Creating debian for $PACKAGE..."

    # We have to go to the ROS package parent directory
    cd $PACKAGE;
    bloom-generate rosdebian --ros-distro foxy &> /dev/null
    fakeroot debian/rules "binary --parallel" &> /dev/null

    cd ..
    DEB_FILE=$(find *.deb);
    dpkg -i $DEB_FILE
    rm *.deb *.ddeb
    cd $CUR_DIR

done

echo "Complete!"
