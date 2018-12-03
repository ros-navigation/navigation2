# nav2_bringup

The `nav2_bringup` package is an example bringup system for navigation2 applications.

Notes: (December 2018, Crystal Release) 
* We recommend doing this on a Ubuntu 18.04 installation. We’re currently having build issues on 16.04. (see https://github.com/ros-planning/navigation2/issues/353)
* This stack and ROS2 are still in heavy development and there are some bugs and stability issues being worked on, so please do not try this on a robot without taking *heavy* safety precautions. THE ROBOT MAY CRASH!
* It is recommended to start with simulation using Gazebo before proceeding to run on a physical robot

Install and build our code by following this guide:
https://github.com/ros-planning/navigation2/blob/master/doc/BUILD.md

## Launch Navigation2 in simulation with Gazebo (first time users)
Pre-requisites:
* Gazebo installed on the system
* gazebo_ros_pkgs for ROS2 installed on the system
* A Gazebo world for simulating the robot (see Gazebo tutorials)
* A map of that world saved to a map.pgm and map.yaml (see ROS Navigation tutorials)

Launch Gazebo and Rviz2

`ros2 launch nav2_bringup gazebo_rviz2_launch.py world:=<full/path/to/gazebo.world>`

Launch your robot specific transforms

Example: See [turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo) for details

`ros2 launch turtlebot3_bringup turtlebot3_robot.launch.py`

Set the tf publisher node to use simulation time or AMCL won't get the transforms correctly

`ros2 param set /robot_state_publisher use_sim_time True`

Launch map_server and AMCL

`ros2 launch nav2_bringup nav2_bringup_1st_launch.py map:=<full/path/to/map.yaml> use_sim_time:=True`

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.

Run the rest of the Navigation2 bringup

`ros2 launch nav2_bringup nav2_bringup_2nd_launch.py use_sim_time:=True`

Set the World Model node to use simulation time

`ros2 param set /world_model use_sim_time True`

Notes:
* Setting use_sim_time has to be done dynamically after the nodes are up due to this bug:https://github.com/ros2/rclcpp/issues/595
* Sometimes setting use_sim_time a second time is required for all the nodes to get updated
* IF you continue to see WARN messages like the ones below, retry setting the use_sim_time parameter
```
[WARN] [world_model]: Costmap2DROS transform timeout. Current time: 1543616767.1026, global_pose stamp: 758.8040, tolerance: 0.3000, difference: 1543616008.2986
[WARN] [FollowPathNode]: Costmap2DROS transform timeout. Current time: 1543616767.2787, global_pose stamp: 759.0040, tolerance: 0.3000, difference: 1543616008.2747
```

In RVIZ:
* Localize the robot using “2D Pose Estimate” button.
* Send the robot a goal using “2D Nav Goal” button.

## Launch Navigation2 on a Robot (first time users)

Pre-requisites:
* Run SLAM or Cartographer with tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done. If not, it can be done in ROS1 before beginning to install our code.
* Publish all the transforms from your robot from base_link to base_scan

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
* Localize the robot using “2D Pose Estimate” button.
* Send the robot a goal using “2D Nav Goal” button.

## Advanced 1-step Launch for experienced users
Pre-requisites:
* You've completed bringup of your robot successfully following the 2-step process above
* You know your transforms are being published correctly and AMCL can localize

Follow directions above *except* 
* Instead of running the `nav2_bringup_1st_launch.py` then the `nav2_bringup_2nd_launch.py`
* You can do it in one step like this:
```
ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml>
```
If running in simulation:
```
ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml> use_sim_time:=True
ros2 param set /world_model use_sim_time True
```

## Future Work

* adding configuration files for the example bringup
* a more complete map for system level testing
