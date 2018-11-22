# nav2_bringup

The `nav2_bringup` package is an example bringup system for navigation2 applications.

## Launch Navigation2 in simulation with Gazebo
Launch Gazebo and Rviz2
 
`ros2 launch nav2_bringup gazebo_rviz2_launch.py world:=<full/path/to/gazebo.world>`

Launch your robot specific transforms

Example: See [turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo) for details

`ros2 launch turtlebot3_bringup turtlebot3_robot.launch.py`

Launch map_server and AMCL
 
`ros2 launch nav2_bringup nav2_bringup_1st_launch.py map:=<full/path/to/map.yaml> use_sim_time:=True`

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.

Run the rest of the Navigation2 bringup

`ros2 launch nav2_bringup nav2_bringup_2nd_launch.py`

In RVIZ:
* Send the robot a goal using “2D Nav Goal” button.

## Launch Navigation2 on a Robot

Pre-requisites:
* Run SLAM or Cartographer with tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done. If not, it can be done in ROS1 before beginning to install our code.
* Publish all the transforms from your robot from base_link to base_scan

Note: We recommend doing this on a Ubuntu 18.04 installation. We’re currently having build issues on 16.04. 

Install and build our code by following this guide:
https://github.com/ros-planning/navigation2/blob/master/doc/BUILD.md

Launch the code using this launch file and your map.yaml:

`ros2 launch nav2_bringup nav2_bringup_1st_launch.py map:=<full/path/to/map.yaml>`

In another terminal, run RVIZ:

`ros2 run rviz2 rviz2`

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.

Run the rest of the Navigation2 bringup

`ros2 launch nav2_bringup nav2_bringup_2nd_launch.py`

In RVIZ:
* Send the robot a goal using “2D Nav Goal” button.

## Future Work

* adding configuration files for the example bringup
* a more complete map for system level testing
