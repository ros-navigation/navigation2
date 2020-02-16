.. _ros1_comparison:

Navigation and Navigation 2 Comparison
######################################

``move_base`` has been split into multiple components.
Rather than a single monolithic state machine, navigation 2 makes use of action servers and ROS2's
low-latency, reliable communication to separate ideas. A behavior tree is used to orchestrate these tasks.
This allows Navigation2 to have highly configurable navigation behavior without programming by rearranging tasks
in a behavior tree xml file.

The *nav2_bt_navigator* replaces move_base at the top level, with an *Action* interface to call the planner and controller with a configurable tree-based action model.

The *nav2_bt_navigator* itself is also an *Action Server* and can also be replaced with other implementations. It uses *Behavior Trees* to make it possible to have more complex state machines and to add in recovery behaviors as additional *Action Servers*.

The planning, recovery, and controller servers are also action servers that the BT navigator can call to compute.
All 3 servers can host many plugins of many algorithms each and individually called from the navigation behavior tree for specific behaviors.
The default plugins provided are ported from ROS1, namely: DWB, NavFn, and similar recoveries such as spinning and clearing costmaps.
A new recovery for waiting a fixed duration was also added.

All these changes make it possible to replace any of these nodes at launch/run time with any other algorithm that implements that same interface.
See each package README.md for more details.

.. image:: images/move_base_compare_2.png
    :width: 700px
    :alt: Move Base -> Navigation2 Overview

**In Summary:**

Ported packages:

  * amcl: Ported to nav2_amcl
  * map_server: Ported to nav2_map_server
  * nav2_planner: Replaces global_planner, hosts ``N`` planner plugins
  * nav2_controller: Replaces local_planner, hosts ``N`` controller plugins
  * Navfn: Ported to nav2_navfn_planner
  * DWB: Replaces DWA and ported to ROS2 under nav2_dwb_controller metapackage
  * nav_core: Ported as nav2_core with updates to interfaces
  * costmap_2d: Ported as nav2_costmap_2d

New packages:

  * nav2_bt_navigator: replaces ``move_base`` state machine
  * nav2_lifecycle_manager: Handles the server program lifecycles
  * nav2_waypoint_follower: Can take in many waypoints to execute a complex task through
  * nav2_system_tests: A set of integration tests for CI and basic tutorials in simulation
  * nav2_rviz_plugins: An rviz plugin to control the Navigation2 servers, command, cancel, and navigation with
  * nav2_experimental: Experimental (and incomplete) work for deep reinforement learning controllers
  * navigation2_behavior_trees: wrappers for the behavior tree library to call ROS action servers
