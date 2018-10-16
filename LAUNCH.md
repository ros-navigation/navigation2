


Start *two* ROS 1 windows, that run these two commands respectively (Also, if needed, export ROS_MASTER_URI=http://localhost:11311 before running the command)
```
    source ros_ws/devel/setup.bash
    roslaunch turtlebot_gazebo turtlebot_world.launch
```
```
    source ros_ws/devel/setup.bash
    roslaunch turtlebot_rviz_launchers view_navigation.launch
```
In a new ROS2 window, start the ROS 1 bridge
```
    source ros2_dev/navigation2_ws/install/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge
```
In a 2nd ROS 2 window
```
    export TEST_LAUNCH_DIR=ros2_ws/navigation2_ws/src/navigation2/nav2_bringup/launch
    ros2 launch nav2_bringup core.launch.py
```

Wait for AMCL to issue the following error "Couldn't transform from /camera_depth_frame to base_footprint..."
Restart the ROS 1 bridge

In RViz:
 - Uncheck the "RobotModel" checkbox
 - Change the Map topic from /map to /occ_grid


 
