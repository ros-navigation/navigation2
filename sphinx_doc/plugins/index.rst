.. _plugins:

Navigation Plugins
##################

There are a number of plugin interfaces for users to create their own custom applications or algorithms with.
Namely, the costmap layer, planner, controller, behavior tree, and recovery plugins.
A list of all known plugins are listed here below for ROS2 Navigation.
If you know of a plugin, or you have created a new plugin, please consider submitting a pull request with that information.

This file can be found and editted under ``sphinx_docs/plugins/index.rst``.
For tutorials on creating your own plugins, please see :ref:`writing_new_costmap2d_plugin`.

Costmap Layers
==============

+--------------------------------+------------------------+----------------------------------+
|            Plugin Name         |         Creator        |       Description                |
+================================+========================+==================================+
| `Voxel Layer`_                 | Eitan Marder-Eppstein  | Maintains persistant             |
|                                |                        | 3D voxel layer using depth and   |
|                                |                        | laser sensor readings and        |
|                                |                        | raycasting to clear free space   |
+--------------------------------+------------------------+----------------------------------+
| `Static Layer`_                | Eitan Marder-Eppstein  | Gets static ``map`` and loads    |
|                                |                        | occupancy information into       |
|                                |                        | costmap                          |
+--------------------------------+------------------------+----------------------------------+
| `Inflation Layer`_             | Eitan Marder-Eppstein  | Inflates lethal obstacles in     |
|                                |                        | costmap with exponential decay   |
+--------------------------------+------------------------+----------------------------------+
|  `Obstacle Layer`_             | Eitan Marder-Eppstein  | Maintains persistent 2D costmap  |
|                                |                        | from 2D laser scans with         |
|                                |                        | raycasting to clear free space   |
+--------------------------------+------------------------+----------------------------------+
| `Spatio-Temporal Voxel Layer`_ |  Steve Macenski        | Maintains temporal 3D sparse     |
|                                |                        | volumetric voxel grid with decay |
|                                |                        | through sensor models            |
+--------------------------------+------------------------+----------------------------------+
| `Non-Persistent Voxel Layer`_  |  Steve Macenski        | Maintains 3D occupancy grid      |
|                                |                        | consisting only of the most      |
|                                |                        | sets of measurements             |
+--------------------------------+------------------------+----------------------------------+

.. _Voxel Layer: https://github.com/ros-planning/navigation2/tree/master/nav2_costmap_2d/plugins
.. _Static Layer: https://github.com/ros-planning/navigation2/tree/master/nav2_costmap_2d/plugins
.. _Inflation Layer: https://github.com/ros-planning/navigation2/tree/master/nav2_costmap_2d/plugins
.. _Obstacle Layer: https://github.com/ros-planning/navigation2/tree/master/nav2_costmap_2d/plugins
.. _Spatio-Temporal Voxel Layer: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/
.. _Non-Persistent Voxel Layer: https://github.com/SteveMacenski/nonpersistent_voxel_layer

Controllers
===========

+--------------------------+--------------------+----------------------------------+
|      Plugin Name         |       Creator      |       Description                |
+==========================+====================+==================================+
|  `DWB Controller`_       | David Lu!!         | A highly configurable  DWA       |
|                          |                    | implementation with plugin       |
|                          |                    | interfaces                       |
+--------------------------+--------------------+----------------------------------+
|  `TEB Controller`_       | Christoph Rösmann  | A MPC-like controller suitable   |
|                          |                    | for ackermann, differential, and |
|                          |                    | holonomic robots.                |
+--------------------------+--------------------+----------------------------------+

.. _DWB Controller: https://github.com/ros-planning/navigation2/tree/master/nav2_dwb_controller
.. _TEB Controller: https://github.com/rst-tu-dortmund/teb_local_planner

Planners
========

+-------------------+---------------------------------------+------------------------------+
| Plugin Name       |         Creator                       |       Description            |
+===================+=======================================+==============================+
|  `NavFn Planner`_ | Eitan Marder-Eppstein & Kurt Konolige | A navigation function        |
|                   |                                       | using A* or Dijkstras        |
|                   |                                       | expansion, assumes 2D        |
|                   |                                       | holonomic particle           |
+-------------------+---------------------------------------+------------------------------+

.. _NavFn Planner: https://github.com/ros-planning/navigation2/tree/master/nav2_navfn_planner

Recoveries
==========

+----------------------+------------------------+----------------------------------+
|  Plugin Name         |         Creator        |       Description                |
+======================+========================+==================================+
|  `Clear Costmap`_    | Eitan Marder-Eppstein  | A service to clear the given     |
|                      |                        | costmap in case of incorrect     |
|                      |                        | perception or robot is stuck     |
+----------------------+------------------------+----------------------------------+
|  `Spin`_             | Steve Macenski         | Rotate recovery of configurable  |
|                      |                        | angles to clear out free space   |
|                      |                        | and nudge robot out of potential |
|                      |                        | local failures                   |
+----------------------+------------------------+----------------------------------+
|    `Back Up`_        | Brian Wilcox           | Back up recovery of configurable |
|                      |                        | distance to back out of a        |
|                      |                        | situation where the robot is     |
|                      |                        | stuck                            |
+----------------------+------------------------+----------------------------------+
|             `Wait`_  | Steve Macenski         | Wait recovery with configurable  |
|                      |                        | time to wait in case of time     |
|                      |                        | based obstacle like human traffic|
|                      |                        | or getting more sensor data      |
+----------------------+------------------------+----------------------------------+

.. _Rotate: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Back Up: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Spin: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Wait: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Clear Costmap: https://github.com/ros-planning/navigation2/blob/master/nav2_costmap_2d/src/clear_costmap_service.cpp

Behavior Tree Nodes
===================

+--------------------------------------------+---------------------+----------------------------------+
| Action Plugin Name                         |   Creator           |       Description                |
+============================================+=====================+==================================+
| `Back Up Action`_                          | Michael Jeronimo    | Calls backup recovery action     |
+--------------------------------------------+---------------------+----------------------------------+
| `Clear Costmap Service`_                   | Carl Delsey         | Calls clear costmap service      |
+--------------------------------------------+---------------------+----------------------------------+
| `Compute Path to Pose Action`_             | Michael Jeronimo    | Calls Nav2 planner server        |
+--------------------------------------------+---------------------+----------------------------------+
| `Follow Path Action`_                      | Michael Jeronimo    | Calls Nav2 controller server     |
+--------------------------------------------+---------------------+----------------------------------+
| `Navigate to Pose Action`_                 | Michael Jeronimo    | BT Node for other                |
|                                            |                     | BehaviorTree.CPP BTs to call     |
|                                            |                     | Navigation2 as a subtree action  |
+--------------------------------------------+---------------------+----------------------------------+
| `Reinitalize Global Localization Service`_ | Carl Delsey         | Reinitialize AMCL to a new pose  |
+--------------------------------------------+---------------------+----------------------------------+
| `Spin Action`_                             | Carl Delsey         | Calls spin recovery action       |
+--------------------------------------------+---------------------+----------------------------------+
| `Wait Action`_                             | Steve Macenski      | Calls wait recovery action       |
+--------------------------------------------+---------------------+----------------------------------+

.. _Back Up Action: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Clear Costmap Service: https://github.com/ros-planning/navigation2/blob/master/nav2_costmap_2d/src/clear_costmap_service.cpp
.. _Compute Path to Pose Action: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Follow Path Action: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Navigate to Pose Action: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Reinitalize Global Localization Service: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Spin Action: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Wait Action: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins


+------------------------------------+--------------------+------------------------+
| Condition Plugin Name              |         Creator    |       Description      |
+====================================+====================+========================+
| `Goal Reached Condition`_          | Carl Delsey        | Checks if goal is      |
|                                    |                    | reached within tol.    |
+------------------------------------+--------------------+------------------------+
| `Initial Pose received Condition`_ | Carl Delsey        | Checks if initial pose |
|                                    |                    | has been set           |
+------------------------------------+--------------------+------------------------+
| `Is Stuck Condition`_              |  Michael Jeronimo  | Checks if robot is     |
|                                    |                    | making progress or     |
|                                    |                    | stuck                  |
+------------------------------------+--------------------+------------------------+
| `Transform Available Condition`_   |  Steve Macenski    | Checks if a TF         |
|                                    |                    | transformation is      |
|                                    |                    | available. When        |
|                                    |                    | succeeds returns       |
|                                    |                    | sucess for subsequent  |
|                                    |                    | calls.                 |
+------------------------------------+--------------------+------------------------+

.. _Goal Reached Condition: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Initial Pose received Condition: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Is Stuck Condition: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Transform Available Condition: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins

+-----------------------+-------------------+----------------------------------+
| Decorator Plugin Name |    Creator        |       Description                |
+=======================+===================+==================================+
| `Rate Controller`_    | Michael Jeronimo  | Throttles child node to a given  |
|                       |                   | rate                             |
+-----------------------+-------------------+----------------------------------+

.. _Rate Controller: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins

+-----------------------+------------------------+----------------------------------+
| Control Plugin Name   |         Creator        |       Description                |
+=======================+========================+==================================+
| `Pipeline Sequence`_  | Carl Delsey            | A variant of a sequence node that|
|                       |                        | will re-tick previous children   |
|                       |                        | even if another child is running |
+-----------------------+------------------------+----------------------------------+
| `Recovery`_           | Carl Delsey            | Node must contain 2 children     |
|                       |                        | and returns success if first     |
|                       |                        | succeeds. If first fails, the    |
|                       |                        | second will be ticked. If        |
|                       |                        | successful, it will retry the    |
|                       |                        | first and then return its value  |
+-----------------------+------------------------+----------------------------------+
| `Round Robin`_        | Mohammad Haghighipanah | Will tick ``i`` th child until   |
|                       |                        | a result and move on to ``i+1``  |
+-----------------------+------------------------+----------------------------------+

.. _Pipeline Sequence: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Recovery: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
.. _Round Robin: https://github.com/ros-planning/navigation2/tree/master/nav2_recoveries/plugins
