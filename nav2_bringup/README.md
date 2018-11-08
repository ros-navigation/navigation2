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

## Future Work

* adding configuration files for the example bringup
* a more complete map for system level testing
