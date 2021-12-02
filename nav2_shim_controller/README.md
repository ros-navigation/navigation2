# Nav2 Shim Controller
The main goal of the Master Controller is to check the angle between the robot&apos;s current orientation and the path.
When the angle is larger than the threshold, the MC will rotate to path orientation.
If the angle is below the threshold the loaded local trajectory plugin (like DWB or TEB) continues functioning.

###Parameters
----
####max_angular_vel
| Type  | Default  |
| :------------ |:---------------:|
| double | 1.5 |
#####Description:
The desired maximum angular velocity ($$\frac {rad}{s}$$) to use.

----
####use_dynamic_threshold
| Type  | Default  |
| :------------ |:---------------:|
| bool | true |
#####Description:
Turns on a dynamic threshold that is proportional to the current linear velocity.
$$threshold = \frac{|current\\_speed|}{max\\_linear\\_vel}$$
Minimum available calculated threshold is ``yaw_goal_tolerance`` of goal checker plugin.
Maximum is ``max_angle_threshold``.

----
####max_angle_threshold
| Type  | Default  |
| :------------ |:---------------:|
| double | 0.785 |
#####Description:
The maximum difference in the path orientation and the starting robot orientation (radians) forces robot return to the path, if ``use_rotate_to_heading`` or ``use_rotate_to_path`` is ``true``.

----
####max_linear_vel
| Type  | Default  |
| :------------ |:---------------:|
| double | 0.26 |
#####Description:
Maximum linear velocity of the default controller used to calculate the dynamic threshold if ``use_dynamic_threshold`` is ``true``

----
####max_angular_accel
| Type  | Default  |
| :------------ |:---------------:|
| double | 1.2 |
#####Description:
Maximum angular acceleration/deceleration ($$\frac {rad}{s^2}$$) when the robot is turning, if ``use_rotate_to_heading`` or ``use_rotate_to_path`` is ``true``.

----
####transform_tolerance
| Type  | Default  |
| :------------ |:---------------:|
| double | 0.1 |
#####Description:
The TF transform tolerance $$\footnotesize (s)$$.

----
####use_rotate_to_heading
| Type  | Default  |
| :------------ |:---------------:|
| bool | true |
#####Description:
Turns on a rotation to goal orientation logic when robot in the XY tolerance.

----
####use_rotate_to_path
| Type  | Default  |
| :------------ |:---------------:|
| bool | true |
#####Description:
It turns on a rotation to goal orientation logic when angle between the robot&apos;s current orientation and the path is larger then the angle threshold.

----
####default_plugin
| Type  | Default  |
| :------------ |:---------------:|
| string | "" |
#####Description:
Default controller which will follow the path when the angle between the robot&apos;s current orientation and the path is lower than the angle threshold.

----
###Example
```
controller_server:
    ros__parameters:
      use_sim_time: True
      controller_frequency: 20.0
      min_x_velocity_threshold: 0.001
      min_y_velocity_threshold: 0.5
      min_theta_velocity_threshold: 0.001
      failure_tolerance: 0.3
      progress_checker_plugin: "progress_checker"
      goal_checker_plugins: ["general_goal_checker"]
      controller_plugins: ["FollowPath"]

      # Progress checker parameters
      progress_checker:
        plugin: "nav2_controller::SimpleProgressChecker"
        required_movement_radius: 0.5
        movement_time_allowance: 10.0

      general_goal_checker:
        stateful: True
        plugin: "nav2_controller::SimpleGoalChecker"
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 0.25

      FollowPath:
        plugin: "nav2_shim_controller::ShimController"
        lookahead_dist: 0.4
        max_angular_vel: 0.8
        max_linear_vel: 0.26
        max_angular_accel: 1.2
        max_angle_threshold: 0.785
        transform_tolerance: 1.0
        use_rotate_to_heading: true
        use_rotate_to_path: true
        use_dynamic_threshold: true
        default_plugin: "FollowPathDefault"

      # DWB parameters
      FollowPathDefault:
        plugin: "dwb_core::DWBLocalPlanner"
        debug_trajectory_details: True
        min_vel_x: 0.0
        min_vel_y: 0.0
        max_vel_x: 0.26
        max_vel_y: 0.0
        max_vel_theta: 1.0
        min_speed_xy: 0.0
        max_speed_xy: 0.26
        min_speed_theta: 0.0
        # Add high threshold velocity for turtlebot 3 issue.
        # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/75
        acc_lim_x: 2.5
        acc_lim_y: 0.0
        acc_lim_theta: 3.2
        decel_lim_x: -2.5
        decel_lim_y: 0.0
        decel_lim_theta: -3.2
        vx_samples: 20
        vy_samples: 5
        vtheta_samples: 20
        sim_time: 1.7
        linear_granularity: 0.05
        angular_granularity: 0.025
        transform_tolerance: 0.2
        xy_goal_tolerance: 0.25
        trans_stopped_velocity: 0.25
        short_circuit_trajectory_evaluation: True
        stateful: True
        critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
        BaseObstacle.scale: 0.02
        PathAlign.scale: 32.0
        PathAlign.forward_point_distance: 0.1
        GoalAlign.scale: 24.0
        GoalAlign.forward_point_distance: 0.1
        PathDist.scale: 32.0
        GoalDist.scale: 24.0
        RotateToGoal.scale: 32.0
        RotateToGoal.slowing_factor: 5.0
        RotateToGoal.lookahead_time: -1.0
```