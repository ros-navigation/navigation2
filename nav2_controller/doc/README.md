# Local Planner

## Local Planner under ROS 1

Under ROS 1, the navigation stack provides a `BaseLocalPlanner` interface
used by `MoveBase`. The `BaseLocalPlanner` component is expected to take a path and current position and produce a command velocity. The most commonly used implementation of the local planner uses the DWA algorithm.

The navigation repository provides two implementations of the DWA algorithm: `base_local_planner` and `dwa_local_planner`. In addition, there is another DWA based planner in the Robot Navigation repository called DWB. The `dwa_local_planner` was meant to replace `base_local_planner` and DWB was meant to replace both the others.

![Local Planner Structure](./images/LocalPlanner.svg "Local planner structure under ROS 1")

## Migrating to ROS 2

Rather than continue with 3 overlapping implementations of DWA, we chose to port the DWB planner to ROS 2 and use it as the standard local planner.

The advantages of DWB were:
* it seemed to be backward compatible with the other Planners
* it was designed with clear extension points
* it has a relatively clean interface

## Changes to DWB

For the most part, the DWB codebase is unchanged, other than what was required for ROS2.

The main architectural change is due to the replacement of `MoveBase` by the `FollowPath` task interface.

![ROS 1 DWB Structure](./images/DWB_Structure_Simplified.svg "ROS 1 DWB Structure")

We basically removed the `MoveBase` adapter layers and `nav_core2` interfaces. Instead, they were replaced by a simple adapter from the `FollowPath` task interface to the `DWBLocalPlanner` component.

## Future Plans

* Simplify parameters and remove obsolete parameters
* Remove the direct inclusion of `costmap_2d` and replace it with calls to the world model node.
* Port another planner to refine the interfaces to the planner and to the world model. There are not many local planners available for the navigation stack, it seems. However `Time Elastic Band` seems like an interesting possibility of the few that are out there, as it interprets the dynamic obstacles very differently.
