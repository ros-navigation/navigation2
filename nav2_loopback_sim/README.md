# Nav2 Loopback Simulation

The Nav2 loopback simulator is a stand-alone simulator to create a "loopback" for non-physical simulation to replace robot hardware, physics simulators (Gazebo, Bullet, Isaac Sim, etc). It computes the robot's odometry based on the command velocity's output request to create a perfect 'frictionless plane'-style simulation for unit testing, system testing, R&D on higher level systems, and testing behaviors without concerning yourself with localization accuracy or system dynamics.

This was created by Steve Macenski of [Open Navigation LLC](https://opennav.org) and donated to Nav2 by the support of our project sponsors. If you rely on Nav2, please consider supporting the project!

**⚠️ If you need professional services related to Nav2, please contact [Open Navigation](https://www.opennav.org/) at info@opennav.org.**

It is drop-in replacable with AMR simulators and global localization by providing:
- Map -> Odom transform
- Odom -> Base Link transform, `nav_msgs/Odometry` odometry 
- Accepts the standard `/initialpose` topic for transporting the robot to another location

Note: This does not provide sensor data, so it is required that the global (and probably local) costmap contain the `StaticLayer` to avoid obstacles.

It is convenient to be able to test systems by being able to:
- Arbitrarily transport the robot to any location and accurately navigate without waiting for a particle filter to converge for testing behaviors and reproducing higher-level issues
- Write unit or system tests on areas that are not dependent on low-level controller or localization performance without needing to spin up a compute-heavy process like Gazebo or Isaac Sim to provide odometry and sensor data, such as global planning, autonomy behavior trees, etc
- Perform R&D on various sensitive systems easily without concerning yourself with the errors accumulated with localization performance or imperfect dynamic models to get a proof of concept started
- When otherwise highly compute constrained and need to simulate a robotic system

## How to Use

```
ros2 run nav2_loopback_sim loopback_simulator  # As a node, if in simulation
ros2 launch nav2_loopback_sim loopback_simulation.launch.py  # As a launch file
ros2 launch nav2_bringup tb3_loopback_simulation.launch.py  # Nav2 integrated navigation demo using it
ros2 launch nav2_bringup tb4_loopback_simulation.launch.py  # Nav2 integrated navigation demo using it
```

## API

### Parameters

- `update_duration`: the duration between updates (default 0.01 -- 100hz)
- `base_frame_id`: The base frame to use (default `base_link`)
- `odom_frame_id`: The odom frame to use (default `odom`)
- `map_frame_id`: The map frame to use (default `map`)
- `scan_frame_id`: The map frame to use (default `base_scan` for TB3, `rplidar_link` for TB4)

### Topics

This node subscribes to:
- `initialpose`: To set the initial robot pose or relocalization request analog to other localization systems
- `cmd_vel`: Nav2's output twist to get the commanded velocity

This node publishes:
- `odom`: To publish odometry from twist
- `tf`: To publish map->odom and odom->base_link transforms
- `scan`: To publish a bogus max range laser scan sensor to make the collision monitor happy
