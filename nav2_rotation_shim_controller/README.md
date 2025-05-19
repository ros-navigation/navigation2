# Nav2 Rotation Shim Controller

This is a controller (local trajectory planner) that implements a "shim" controller plugin. It was developed by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research](https://www.sra.samsung.com/). 

The Rotation Shim Controller stands between the controller server and the main controller plugin to implement a specific behavior often troublesome for other algorithms. This shim will rotate a robot in place to the rough heading of a newly received path. Afterwards, it will forward all future commands on that path to the main controller. It will take in a ``primary_controller`` parameter, containing the actual controller to use for path tracking once aligned with the path. 

This is useful for situations when working with plugins that are either too specialized or tuned for a particular task that they can fail to adequately solve the full local planning problem performantly. Examples include:

- Heavily tuning DWB for excellent path tracking makes it difficult to handle large deviations
- TEB behavior tends to "whip" the robot around with small turns, in a somewhat scary way due to the elastic band approach
- Neither TEB or DWB will simply rotate the robot in place to start tracking a new path. They instead perform a small 'spiral out' maneuver that is often clunky with odd velocities. You may prefer a clean and simple rotation in place.

As such, this controller will check the rough heading difference with respect to the robot and a newly received path. If within a threshold, it will pass the request onto the controller to execute. If it is outside of the threshold, this controller will rotate the robot towards that path heading. Once it is within the tolerance, it will then pass off control-execution from this rotation shim controller onto the primary controller plugin. At this point, the robot is still going to be rotating, allowing the current plugin to take control for a smooth hand off into path tracking. It is recommended to be more generous than strict in the angular threshold to allow for a smoother transition, but should be tuned for a specific application's desired behaviors.

When the `rotate_to_goal_heading` parameter is set to true, this controller is also able to take back control of the robot when reaching the XY goal tolerance of the goal checker. In this case, the robot will rotate towards the goal heading until the goal checker validate the goal and ends the current navigation task.

The Rotation Shim Controller is suitable for:
- Robots that can rotate in place, such as differential and omnidirectional robots.
- Preference to rotate in place rather than 'spiral out' when starting to track a new path that is at a significantly different heading than the robot's current heading.
- Using planners that are non-kinematically feasible, such as NavFn, Theta\*, or Smac 2D (Feasible planners such as Smac Hybrid-A* and State Lattice will start search from the robot's actual starting heading, requiring no rotation). 

This plugin implements the `nav2_core::Controller` interface allowing it to be used across the navigation stack as a local trajectory planner in the controller server's action server (`controller_server`). It will host an internal plugin of your actual path tracker (e.g. MPPI, RPP, DWB, TEB, etc) that will be used after the robot has rotated to the rough starting heading of the path.

<p align="center">
  <img src="https://user-images.githubusercontent.com/14944147/144323291-29e24521-674a-41f5-8a91-732121b26b47.gif">
</p>

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html) for additional parameter descriptions.

## Configuration

| Parameter | Description | 
|-----|----|
| `angular_dist_threshold` | Maximum angular distance, in radians, away from the path heading to trigger rotation until within. |
| `forward_sampling_distance` | Forward distance, in meters, along path to select a sampling point to use to approximate path heading |
| `rotate_to_heading_angular_vel` | Angular rotational velocity, in rad/s, to rotate to the path heading |
| `primary_controller` | Internal controller plugin to use for actual control behavior after rotating to heading |
| `max_angular_accel` | Maximum angular acceleration for rotation to heading |
| `simulate_ahead_time` | Time in seconds to forward simulate a rotation command to check for collisions. If a collision is found, forwards control back to the primary controller plugin. |
| `rotate_to_goal_heading` | If true, the rotationShimController will take back control of the robot when in XY tolerance of the goal and start rotating to the goal heading |
| `use_path_orientations` | If true, the controller will use the orientations of the path points to compute the heading of the path instead of computing the heading from the path points. If true, the controller will use the orientations of the path points to compute the heading of the path instead of computing the heading from the path points. Use for for feasible planners like the Smac Planner which generate feasible paths with orientations for forward and reverse motion. |

Example fully-described XML with default parameter values:

```
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: "goal_checker"
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "dwb_core::DWBLocalPlanner"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: false
      use_path_orientations: false

      # DWB parameters
      ...
      ...
      ...
```
