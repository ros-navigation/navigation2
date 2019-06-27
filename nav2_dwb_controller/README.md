# Local Planner

## Local Planner under ROS 1

Under ROS 1, the navigation stack provides a `BaseLocalPlanner` interface
used by `MoveBase`. The `BaseLocalPlanner` component is expected to take a path and current position and produce a command velocity. The most commonly used implementation of the local planner uses the DWA algorithm, however, the Trajectory Rollout algorithm is also available in the ROS 1 navigation stack.

The navigation repository provides two implementations of the DWA algorithm: `base_local_planner` and `dwa_local_planner`. In addition, there is another DWA based planner in the [Robot Navigation repository](https://github.com/locusrobotics/robot_navigation) called DWB. The `dwa_local_planner` was meant to replace `base_local_planner` and provides a better DWA algorithm, but failed to
provide the Trajectory Rollout algorithm, so was unable to completely replace
`base_local_planner`.

DWB was meant to replace both the planners in the navigation repository and provides
an implementation of both DWA and Trajectory Rollout.

![Local Planner Structure](./images/LocalPlanner.svg "Local planner structure under ROS 1")

## Migrating to ROS 2

Rather than continue with 3 overlapping implementations of DWA, the DWB planner
was chosen to use as the standard local planner.

The advantages of DWB were:
* it was designed with backwards compatibility with the other controllers
* it was designed with clear extension points and modularity

## Changes to DWB

For the most part, the DWB codebase is unchanged, other than what was required for ROS2.

The main architectural change is due to the replacement of `MoveBase` by the `FollowPath` task interface.

![ROS 1 DWB Structure](./images/DWB_Structure_Simplified.svg "ROS 1 DWB Structure")

To support the new FollowPath task interface, the `MoveBase` adapter layer and `nav_core2` interfaces were removed. They were replaced by an an alternate adapter between
the `FollowPath` task interface and the `DWBLocalPlanner` component.

## New local planner interface

For the local planner, the new task interface consist of
initialization/destruction code and one core method - `execute`. This method
gets called with the path produced by the global planner. The local planner
keeps processing the path until:
1. It reaches the goal.
2. It gets a new plan which preempts the current plan.
3. It fails to produce an adequate plan, in which case it fails and leaves it to the caller to hand off to a recovery behavior.

This is in contrast to the local planner in ROS1, where the local planner is
queried at each iteration for a velocity command and the iteration and
publishing of the command were handled by `MoveBase`

The new task interface looks something like this in pseudocode
```c++
TaskStatus execute(path)
{
    planner_.initialize(nodeHandle);
    planner_.setPlan(path);
    while (true) {
      auto pose = getRobotPose(pose2d);
      if (isGoalReached(pose)) {
        break;
      }
      auto velocity = getTwist();
      auto cmd_vel_2d = planner_.computeVelocityCommands(pose, velocity);
      publishVelocity(cmd_vel_2d);

      std::this_thread::sleep_for(10ms);
    }

  nav2_behavior_tree::FollowPathResult result;
  setResult(result);

  return TaskStatus::SUCCEEDED;
}

```
## Future Plans

* Add support for recovery behaviors https://github.com/ros-planning/navigation2/issues/49
* Simplify parameters and remove obsolete parameters https://github.com/ros-planning/navigation2/issues/201
* Remove the direct inclusion of `costmap_2d` and replace it with calls to the world model node. https://github.com/ros-planning/navigation2/issues/21
* Port another planner to refine the interfaces to the planner and to the world model. There are not many local planners available for the navigation stack, it seems. However `Time Elastic Band` seems like an interesting possibility of the few that are out there, as it interprets the dynamic obstacles very differently. https://github.com/ros-planning/navigation2/issues/202
