# nav2_bringup

The `nav2_bringup` package is an example bringup system for Navigation2 applications.

### Pre-requisites:
* [Install ROS 2](https://index.ros.org/doc/ros2/Installation/Dashing/)
* Install Navigation2

    ```sudo apt install ros-<ros2_distro>-navigation2```
    
* Install Navigation2 Bringup

    ```sudo apt install ros-<ros2_distro>-nav2-bringup```

* Install your robot specific package (ex:[Turtlebot 3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/))

## Launch Navigation2 in Simulation with Gazebo
### Pre-requisites:

* [Install Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
* gazebo_ros_pkgs for ROS2 installed on the system
    
    ```sudo apt-get install ros-<ros2-distro>-gazebo*```
* A Gazebo world for simulating the robot ([Gazebo tutorials](http://gazebosim.org/tutorials?tut=quick_start))
* A map of that world saved to a map.pgm and map.yaml ([ROS Navigation Tutorials](https://github.com/ros-planning/navigation2/tree/master/doc/use_cases))

### Terminal 1: Launch Gazebo

Example: See [turtlebot3_gazebo models](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo/models) for details

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<full/path/to/my_robot/models>
gazebo --verbose -s libgazebo_ros_init.so <full/path/to/my_gazebo.world>
```

### Terminal 2: Launch your robot specific transforms

Example: See [turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo) for details

```
source /opt/ros/dashing/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True
```

### Terminal 3: Launch Navigation2

```
source /opt/ros/dashing/setup.bash
ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True autostart:=True \
map:=<full/path/to/map.yaml>
```

### Terminal 4: Run RViz with Navigation2 config file
```
source /opt/ros/dashing/setup.bash
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

### Pre-requisites:
* Run SLAM with Navigation 2 or tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done or there is already a map of the area. 

* Learn more about how to use Navigation 2 with SLAM to create maps; 

    - [Navigation 2 with SLAM](https://github.com/ros-planning/navigation2/blob/master/doc/use_cases/navigation_with_slam.md)

* _Please note that currently, nav2_bringup works if you provide a map file. However, providing a map is not required to use Navigation2. Navigation2 can be configured to use the costmaps to navigate in an area without using a map file_

* Publish all the transforms from your robot from base_link to base_scan


### Terminal 1 : Launch Navigation2 using your map.yaml

```
source /opt/ros/dashing/setup.bash
ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml> map_type:=occupancy
```

### Terminal 2 : Launch RVIZ

```
source /opt/ros/dashing/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz
```

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.
* Send the robot a goal pose using “2D Nav Goal” button.
