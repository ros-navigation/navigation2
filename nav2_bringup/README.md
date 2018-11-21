# nav2_bringup

The `nav2_bringup` package is an example bringup system for navigation2 applications.

## Launch Navigation2 in simulation with Gazebo
 - Launch Gazebo and Rviz2
 
```
ros2 launch nav2_bringup gazebo_rviz2_launch.py world:=<full/path/to/gazebo.world>
```
 - Launch Navigation2
 
```
ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml use_sim_time:=True 
```
 - Set 'use\_sim\_time' parameter for static transforms
* This is due to a bug in static\_transform\_publisher - [https://github.com/ros2/geometry2/issues/80](https://github.com/ros2/geometry2/issues/80)

```
ros2 param set /static_transform_publisher use_sim_time True
```

## system_test.rviz

There is an rviz configuration for testing base navigation2 systems.

## map

There is also an example map (pgm and yaml) for system level tests.

## Launch Navigation2 on a Robot

Pre-requisites:
-	You will need to run SLAM or Cartographer with tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done. If not, it can be done in ROS1 before beginning to install our code.

Note: We recommend doing this on a Ubuntu 18.04 installation. We’re currently having build issues on 16.04. 

Install and build our code by following this guide:
https://github.com/ros-planning/navigation2/blob/master/doc/BUILD.md

Launch the code using this launch file and your map.yaml:

`ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml>`

In another terminal, run RVIZ:

`ros2 run rviz2 rviz2`

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.
* Send it a goal using “2D Nav Goal” button.

## Future Work

* adding configuration files for the example bringup
* a more complete map for system level testing
