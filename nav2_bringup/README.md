# nav2_bringup

The `nav2_bringup` package is an example bringup system for Nav2 applications.

This is a very flexible example for nav2 bringup that can be modified for different maps/robots/hardware/worlds/etc. It is our expectation for an application specific robot system that you're mirroring `nav2_bringup` package and modifying it for your specific maps/robots/bringup needs. This is an applied and working demonstration for the default system bringup with many options that can be easily modified.

Usual robot stacks will have a `<robot_name>_nav` package with config/bringup files and this is that for the general case to base a specific robot system off of.

Dynamically composed bringup (based on  [ROS2 Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)) is optional for users. It can be used to compose all Nav2 nodes in a single process instead of launching these nodes separately, which is useful for embedded systems users that need to make optimizations due to harsh resource constraints. Dynamically composed bringup is used by default, but can be disabled by using the launch argument `use_composition:=False`.

* Some discussions about performance improvement of composed bringup could be found here: https://discourse.ros.org/t/nav2-composition/22175.

To use, please see the Nav2 [Getting Started Page](https://docs.nav2.org/getting_started/index.html) on our documentation website. Additional [tutorials will help you](https://docs.nav2.org/tutorials/index.html) go from an initial setup in simulation to testing on a hardware robot, using SLAM, and more.

Note:
* gazebo should be started with both libgazebo_ros_init.so and libgazebo_ros_factory.so to work correctly.
* spawn_entity node could not remap /tf and /tf_static to tf and tf_static in the launch file yet, used only for multi-robot situations. Instead it should be done as remapping argument <remapping>/tf:=tf</remapping>  <remapping>/tf_static:=tf_static</remapping> under ros2 tag in each plugin which publishs transforms in the SDF file. It is essential to differentiate the tf's of the different robot.

## Launch

### Multi-robot Simulation

This is how to launch multi-robot simulation with simple command line. Please see the Nav2 documentation for further augments.

#### Cloned

This allows to bring up multiple robots, cloning a single robot N times at different positions in the map. The parameter are loaded from `nav2_multirobot_params_all.yaml` file by default.
The multiple robots that consists of name and initial pose in YAML format will be set on the command-line. The format for each robot is `robot_name={x: 0.0, y: 0.0, yaw: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}`.

Please refer to below examples.

```shell
ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py robots:="robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: 1.0, y: 1.0, yaw: 1.5707}"
```

#### Unique

There are two robots including name and intitial pose are hard-coded in the launch script. Two separated unique robots are required params file (`nav2_multirobot_params_1.yaml`, `nav2_multirobot_params_2.yaml`) for each robot to bring up.

If you want to bringup more than two robots, you should modify the `unique_multi_tb3_simulation_launch.py` script.

```shell
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py
```
