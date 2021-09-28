# nav2_bringup

The `nav2_bringup` package is an example bringup system for Nav2 applications. 

This is a very flexible example for nav2 bring up that can be modified for different maps/robots/hardware/worlds/etc. It is our expectation for an application specific thing you're mirroring `nav2_bringup` package and modifying it for your specific  maps/robots/bringup needs. So this is an applied and working demonstration for the default system bringup with many options that can be easily modified. 

Usual robot stacks will have a `<robot_name>_nav` package with config/bringup files and this is that for the general case to base a specific robot system off of.

Composed bringup (based on  [ROS2 Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html) ) is optional for user, in other word, compose all Nav2 nodes in a single process instead of launching these nodes separately, which is useful for embedded systems users that need to make optimizations due to harsh resource constraints.  

* some discussions about performance improvement of composed bringup could be found here: https://discourse.ros.org/t/nav2-composition/22175.
* Currently, manual composition is used in this package. Dynamic composition is more flexible than manual composition, but is not currently applied in nav2 due to various issues, you could find more details here: https://github.com/ros-planning/navigation2/issues/2147.

### Pre-requisites:
* [Install ROS 2](https://index.ros.org/doc/ros2/Installation/Dashing/)
* Install Nav2

    ```sudo apt install ros-<ros2_distro>-navigation2```

* Install Nav2 Bringup

    ```sudo apt install ros-<ros2_distro>-nav2-bringup```

* Install your robot specific package (ex:[Turtlebot 3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/))

## Launch Nav2 in *Simulation* with Gazebo
### Pre-requisites:

* [Install Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
* gazebo_ros_pkgs for ROS2 installed on the system

    ```sudo apt-get install ros-<ros2-distro>-gazebo*```
* A Gazebo world for simulating the robot ([Gazebo tutorials](http://gazebosim.org/tutorials?tut=quick_start))
* A map of that world saved to a map.pgm and map.yaml ([ROS Navigation Tutorials](https://github.com/ros-planning/navigation2/tree/main/doc/use_cases))

### Terminal 1: Launch Gazebo

Example: See [turtlebot3_gazebo models](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo/models) for details

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<full/path/to/my_robot/models>
gazebo --verbose -s libgazebo_ros_init.so <full/path/to/my_gazebo.world>
```

### Terminal 2: Launch your robot specific transforms

Example: See [turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo) for details

```bash
source /opt/ros/dashing/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True
```

### Terminal 3: Launch Nav2

normal bringup

```bash
source /opt/ros/dashing/setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True \
map:=<full/path/to/map.yaml>
```

manually composed bringup

```bash
source /opt/ros/dashing/setup.bash
ros2 launch nav2_bringup composed_bringup_launch.py use_sim_time:=True autostart:=True \
map:=<full/path/to/map.yaml>
```

### Terminal 4: Run RViz with Nav2 config file

```bash
source /opt/ros/dashing/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz
```

In RViz:
* You should see the map
* Localize the robot using “2D Pose Estimate” button.
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Send the robot a goal using "Nav2 Goal” button.
Note: this uses a ROS2 Action to send the goal, and a pop-up window will appear on your screen with a 'cancel' button if you wish to cancel

To view the robot model in RViz:
* Add "RobotModel", set "Description Source" with "File", set "Description File" with the name of the urdf file for your robot (example: turtlebot3_burger.urdf)"

### Advanced: single-terminal launch

A convenience file is provided to launch Gazebo, RVIZ and Nav2 using a single command:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py <settings>
```

Where `<settings>` can used to replace any of the default options, for example:

```
use_composition:=<True or False>
world:=<full/path/to/gazebo.world>
map:=<full/path/to/map.yaml>
rviz_config_file:=<full/path/to/rviz_config.rviz>
simulator:=<gzserver or gazebo>
bt_xml_file:=<full/path/to/bt_tree.xml>
```


Before running the command make sure you are sourcing the `ROS2` workspace, setting the path to the Gazebo model and defining the TB3 robot model to use.

```bash
source <full/path/to/ros2/setup.bash>
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<full/path/to/my_robot/models>
export TURTLEBOT3_MODEL=waffle
```

Also, a file for launching **two** robots with **independent** navigation stacks is provided:

```bash
ros2 launch nav2_bringup multi_tb3_simulation_launch.py <settings>
```


## Launch Nav2 on a *Robot*

### Pre-requisites:
* Run SLAM with Navigation 2 or tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done or there is already a map of the area.

* Learn more about how to use Navigation 2 with SLAM to create maps;

    - [Navigation 2 with SLAM](https://github.com/ros-planning/navigation2/blob/main/doc/use_cases/navigation_with_slam.md)

* _Please note that currently, nav2_bringup works if you provide a map file. However, providing a map is not required to use Nav2. Nav2 can be configured to use the costmaps to navigate in an area without using a map file_

* Publish all the transforms from your robot from base_link to base_scan


### Terminal 1 : Launch Nav2 using your map.yaml

```bash
source /opt/ros/dashing/setup.bash
ros2 launch nav2_bringup bringup_launch.py map:=<full/path/to/map.yaml> map_type:=occupancy
```

### Terminal 2 : Launch RVIZ

```bash
source /opt/ros/dashing/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz
```

In RVIZ:
* Make sure all transforms from odom are present. (odom->base_link->base_scan)
* Localize the robot using “2D Pose Estimate” button.
* Send the robot a goal pose using “2D Nav Goal” button.

Note:
* nav2_gazebo_spawner pkg inside nav2_bringup directory is deleted.
* use of nav2_gazebo_spawner to spawn the robot in gazebo is not recommended any more. Instead use spawn_entity.py of gazebo_ros to spawn the robot.
* gazebo should be started with both libgazebo_ros_init.so and libgazebo_ros_factory.so to work correctly.
* spawn_entity node could not remap /tf and /tf_static to tf and tf_static in the launch file yet, used only for multi-robot situations. Instead it should be done as remapping argument <remapping>/tf:=tf</remapping>  <remapping>/tf_static:=tf_static</remapping> under ros2 tag in each plugin which publishs transforms in the SDF file. It is essential to differentiate the tf's of the different robot.