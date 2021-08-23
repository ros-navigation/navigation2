# nav2_compostion

The `nav2_compistion` package is a composition-based bringup for Nav2, which is useful for embedded systems users that need to make optimizations due to harsh resource constraints.

### Pre-requisites:

- [Install ROS 2](https://index.ros.org/doc/ros2/Installation/Dashing/)

- Install Nav2

  `sudo apt install ros-<ros2_distro>-navigation2`

- Install Nav2 Bringup

  `sudo apt install ros-<ros2_distro>-nav2-bringup`

- Install your robot specific package (ex:[Turtlebot 3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/))

## Launch Nav2 in *Simulation* with Gazebo

refer to package `nav2_bringup`,  there is only difference when you run `Terminal 3: Launch Nav2` , which just replaces `bringup_launch.py` with  `compostion_bringup_launch.py`.

```bash
source /opt/ros/dashing/setup.bash
ros2 launch nav2_compostion compostion_bringup_launch.py use_sim_time:=True autostart:=True \
map:=<full/path/to/map.yaml>
```

### Advanced: single-terminal launch

A convenience file is provided to launch Gazebo, RVIZ and Nav2 using a single command:

```bash
ros2 launch nav2_compostion tb3_simulation_launch.py
```



