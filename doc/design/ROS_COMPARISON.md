# Navigation 2 System Overview

## Comparison to Move Base in ROS
AMCL and map_server were ported to ROS2 with minimal functional changes, but some refactoring.

  * amcl -> nav2_amcl
  * map_server -> nav2_map_server

![Move Base 1](./move_base_compare_1.png)

In addition, move_base itself has been split into multiple components:

  * nav2_simple_navigator (replaces move_base)
  * nav2_dijkstra_planner (replaces global_planner)
  * nav2_controller_dwb (replaces local_planner)

![Move Base 2](./move_base_compare_2.png)

The *nav2_simple_navigator* replaces move_base at the top level, with a *Task* interface to call the global and local planners.

* Note: the *Task* interface is a temporary proxy for ROS2 *Actions* which are not yet implemented. When *Actions* become available, the planners will be called through ROS2 *Actions*.

The reason for the change was to make it so that global and local planners would be *Action Servers* and could be replaced at launch or run time with other implementations providing the same *Action*.

The *nav2_simple_navigator* itself is also an *Task Server* and can also be replaced with other implementations. The first such implementation is currently in progress and uses *Behavior Trees* to make it possible to have more complex state machines and to add in recovery behaviors as additional *Task Servers*.

The *nav2_dijkstra_planner* is ported from the *navfn* package in ROS, but adds the *Task Server* interface to enable it to be strongly decoupled from the nav2_simple_navigator.

Similarly, the *nav2_controller_dwb* is ported from the [dwb controller](https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner) [ackage, and also adds a *Task Server* interface to also enable it to be decoupled from the *nav2_simple_navigator*.

All these changes make it possible to replace any of these nodes at launch/run time with any other algorithm that implements that same interface.

**See each package README.md for more details**




