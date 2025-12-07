# Symmetric Goal Angle Critic for MPPI Controller

## Overview

The **SymmetricGoalAngleCritic** is a custom MPPI controller critic designed for symmetric robots that can drive equally well in both forward and backward directions. This critic prevents unnecessary 180° rotations at goal positions by accepting either the goal orientation or the goal orientation + 180°.

## Use Case

This critic is ideal for robots with symmetric mechanical designs, such as:
- Differential drive robots with sensors on both ends
- Robots with bidirectional capabilities
- Mobile platforms where forward and backward motion have equivalent performance

Without this critic, a standard goal angle critic would force the robot to rotate 180° if it approaches the goal from the "wrong" direction, even when the robot could simply drive backward to the goal.

## How It Works

The SymmetricGoalAngleCritic calculates the angular distance to **both** the goal orientation and the flipped goal orientation (goal + 180°), then uses the **minimum** of these two distances for trajectory scoring. This allows the robot to:

1. Approach the goal from either direction without penalty
2. Avoid unnecessary rotations when already facing away from the goal orientation
3. Select the most efficient trajectory based on current orientation

### Algorithm

For each trajectory point:
```
goal_yaw_flipped = normalize_angle(goal_yaw + π)
distance_forward = |shortest_angular_distance(trajectory_yaw, goal_yaw)|
distance_backward = |shortest_angular_distance(trajectory_yaw, goal_yaw_flipped)|
min_distance = min(distance_forward, distance_backward)
cost += (mean(min_distance) × weight)^power
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_weight` | double | 3.0 | Weight multiplier for trajectory cost |
| `cost_power` | int | 1 | Power applied to the weighted cost (higher = sharper cost curve) |
| `threshold_to_consider` | double | 0.5 | Distance threshold (meters) from goal to activate this critic |

### Parameter Details

- **cost_weight**: Higher values increase the importance of orientation alignment relative to other critics. Typical range: 1.0-10.0
- **cost_power**: Exponential power applied to cost. Power of 1 gives linear scaling, 2 gives quadratic, etc.
- **threshold_to_consider**: The critic only activates when the robot is within this distance from the goal position

## Configuration Example

### MPPI Controller Parameters (YAML)

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

      # Motion model parameters
      motion_model: "DiffDrive"

      # Critic configuration
      critics:
        - "ConstraintCritic"
        - "ObstaclesCritic"
        - "GoalCritic"
        - "PathAlignCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
        - "SymmetricGoalAngleCritic"  # Custom symmetric goal critic

      # SymmetricGoalAngleCritic parameters
      SymmetricGoalAngleCritic:
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 0.4

      # Other critic weights
      ConstraintCritic:
        cost_weight: 4.0
      GoalCritic:
        cost_weight: 5.0
        threshold_to_consider: 1.4
      ObstaclesCritic:
        cost_weight: 8.0
      PathAlignCritic:
        cost_weight: 10.0
      PathAngleCritic:
        cost_weight: 2.0
      PreferForwardCritic:
        cost_weight: 5.0
```

### Behavior Tree XML Configuration

Include the MPPI controller in your behavior tree:

```xml
<BehaviorTree ID="FollowPath">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="1" name="FollowPath">
        <PipelineSequence name="FollowPathSequence">
          <FollowPath
            server_name="follow_path"
            server_timeout="10"
            controller_id="FollowPath"
            goal_checker_id="symmetric_goal_checker"/>
        </PipelineSequence>
        <ClearEntireCostmap name="ClearCostmapRecovery" service_name="local_costmap/clear_entirely_global_costmap"/>
      </RecoveryNode>
    </RateController>
  </PipelineSequence>
</BehaviorTree>
```

### Complete Controller Server Configuration

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 30.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"

    # Goal checker - use symmetric goal checker for symmetric robots
    goal_checker_plugins: ["symmetric_goal_checker"]

    symmetric_goal_checker:
      plugin: "nav2_controller::SymmetricGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.15
      stateful: true

    # Controller plugin
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      ax_max: 3.0
      ax_min: -3.0
      ay_max: 0.0
      az_max: 3.5

      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.0
      wz_max: 1.9

      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      regenerate_noises: false

      # Critic list including symmetric goal angle critic
      critics:
        - "ConstraintCritic"
        - "ObstaclesCritic"
        - "GoalCritic"
        - "PathAlignCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
        - "SymmetricGoalAngleCritic"

      # Critic parameters
      ConstraintCritic:
        cost_weight: 4.0
        cost_power: 1

      GoalCritic:
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 1.4

      ObstaclesCritic:
        cost_weight: 8.0
        consider_footprint: false
        collision_cost: 10000.0
        collision_margin_distance: 0.1
        near_goal_distance: 0.5
        inflation_radius: 0.55
        cost_scaling_factor: 10.0

      PathAlignCritic:
        cost_weight: 10.0
        cost_power: 1
        threshold_to_consider: 0.5
        offset_from_path: 1.0
        max_path_occupancy_ratio: 0.05

      PathAngleCritic:
        cost_weight: 2.0
        cost_power: 1
        offset_from_path: 0.4
        threshold_to_consider: 0.5
        max_angle_to_path: 1.0

      PreferForwardCritic:
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 0.5

      # Symmetric goal angle critic parameters
      SymmetricGoalAngleCritic:
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 0.4
```

## Complementary Plugin: SymmetricGoalChecker

For complete symmetric robot support, use the **SymmetricGoalChecker** plugin alongside this critic:

```yaml
goal_checker_plugins: ["symmetric_goal_checker"]

symmetric_goal_checker:
  plugin: "nav2_controller::SymmetricGoalChecker"
  xy_goal_tolerance: 0.15      # XY position tolerance (meters)
  yaw_goal_tolerance: 0.15     # Orientation tolerance (radians)
  stateful: true               # Lock XY check once satisfied
```

The SymmetricGoalChecker accepts the goal as reached when the robot is within tolerance of either:
- The goal orientation, OR
- The goal orientation + 180°

## Building and Installation

### Prerequisites

- ROS 2 (Humble, Iron, or Rolling)
- Nav2 navigation stack
- angles library

### Build Instructions

1. The critic is part of the Nav2 MPPI controller package:
```bash
cd ~/ros2_ws
colcon build --packages-select nav2_mppi_controller
```

2. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Plugin Registration

The critic is automatically registered via pluginlib. Verify registration:

```bash
ros2 pkg plugins --package nav2_mppi_controller
```

You should see `mppi::critics::SymmetricGoalAngleCritic` in the output.

## Tuning Guidelines

### Starting Point
Begin with these values and adjust based on robot behavior:
- `cost_weight`: 5.0
- `cost_power`: 1
- `threshold_to_consider`: 0.4 (40cm from goal)

### Increasing Orientation Precision
If the robot doesn't align well with the goal:
- Increase `cost_weight` (try 7.0-10.0)
- Increase `cost_power` to 2 for sharper cost gradients

### Allowing More Flexibility
If the robot is too aggressive about orientation:
- Decrease `cost_weight` (try 2.0-4.0)
- Keep `cost_power` at 1
- Increase `threshold_to_consider` to activate critic closer to goal

### Performance Considerations
- Lower `cost_weight` values allow other critics (obstacles, path following) to dominate
- Higher `cost_power` creates steeper cost curves but may reduce trajectory diversity
- `threshold_to_consider` should be set based on your robot's turning radius

## Debugging

Enable debug logging to see critic behavior:

```bash
ros2 run nav2_mppi_controller controller_server --ros-args --log-level nav2_mppi_controller:=debug
```

### Expected Behavior
- Critic should only activate when within `threshold_to_consider` of the goal
- At goal approach, trajectories should prefer minimal angular correction
- Robot should accept goals when facing either direction (±180°)

### Common Issues

**Robot still rotates unnecessarily:**
- Increase `cost_weight` to make orientation more important
- Verify SymmetricGoalChecker is being used as the goal_checker
- Check that PreferForwardCritic weight isn't too high

**Robot doesn't align at goal:**
- Decrease `threshold_to_consider` to activate critic earlier
- Increase `cost_power` for sharper cost gradients
- Verify goal checker tolerance isn't too loose

## Source Code

- **Header**: `nav2_mppi_controller/include/nav2_mppi_controller/critics/symmetric_goal_angle_critic.hpp`
- **Implementation**: `nav2_mppi_controller/src/critics/symmetric_goal_angle_critic.cpp`
- **Plugin Registration**: `nav2_mppi_controller/critics.xml`

## License

Licensed under the Apache License, Version 2.0

## Related Plugins

- **SymmetricGoalChecker**: Goal checker that accepts both forward and backward goal orientations
- **GoalAngleCritic**: Standard goal angle critic (only accepts specified goal orientation)
- **GoalCritic**: Position-based goal critic (no orientation consideration)

## References

- [Nav2 MPPI Controller Documentation](https://navigation.ros.org/configuration/packages/configuring-mppi.html)
- [Nav2 Goal Checker Plugins](https://navigation.ros.org/configuration/packages/configuring-controller-server.html#goal-checker-plugins)
- [MPPI Algorithm Paper](https://ieeexplore.ieee.org/document/6386025)
