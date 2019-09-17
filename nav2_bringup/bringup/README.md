# nav2_bringup

The `nav2_bringup` package is an example bringup system for navigation2 applications.

Notes: (June 2019, Dashing Release)
* We recommend doing this on a Ubuntu 18.04 installation. We have build issues on 16.04. (see https://github.com/ros-planning/navigation2/issues/353)
* This stack and ROS2 are still in heavy development and there are some bugs and stability issues being worked on, so please do not try this on a robot without taking *heavy* safety precautions. THE ROBOT MAY CRASH!
* It is recommended to start with simulation using Gazebo before proceeding to run on a physical robot

Install and build our code by following this guide:
https://github.com/ros-planning/navigation2/blob/master/doc/BUILD.md

## Launch Navigation2 in simulation with Gazebo
### Pre-requisites:
* Gazebo installed on the system
* gazebo_ros_pkgs for ROS2 installed on the system
* A Gazebo world for simulating the robot (see Gazebo tutorials)
* A map of that world saved to a map.pgm and map.yaml (see ROS Navigation tutorials)

### Terminal 1: Launch Gazebo

Example: See [turtlebot3_gazebo models](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo/models) for details

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<full/path/to/my_robot/models>
gazebo --verbose -s libgazebo_ros_init.so <full/path/to/my_gazebo.world>
```

### Terminal 2: Launch your robot specific transforms

Example: See [turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo) for details

```
source turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True
```

### Terminal 3: Launch navigation2

```
source navigation2_ws/install/setup.bash
# Launch the nav2 system
ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=True \
map:=<full/path/to/map.yaml>
```

### Terminal 4: Run RViz with navigation2 config file
```
source navigation2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz
```
In RViz:
* You should see the map
* Localize the robot using “2D Pose Estimate” button.
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Send the robot a goal using “Navigation2 Goal” button.
Note: this uses a ROS2 Action to send the goal, and a pop-up window will appear on your screen with a 'cancel' button if you wish to cancel

To view the robot model in RViz:
* Add "RobotModel", set "Description Source" with "File", set "Description File" with the name of the urdf file for your robot (example: turtlebot3_burger.urdf)"

## Launch Navigation2 on a Robot

Pre-requisites:
* Run SLAM or Cartographer with tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done. If not, it can be done in ROS1 before beginning to install our code.
* Publish all the transforms from your robot from base_link to base_scan

Launch the code using this launch file and your map.yaml:

`ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml> map_type:=occupancy`

In another terminal, run RVIZ:

`ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz`

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.

In RVIZ:
* Localize the robot using “2D Pose Estimate” button.
* Send the robot a goal using “2D Nav Goal” button.

## Future Work

* Add instructions for running navigation2 with SLAM