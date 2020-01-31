.. _configuring_the_controller:

Configuring The Controller
##########################

Controller Server
=================

The controller server responds to FollowPath action requests. It runs a loop
that continuously computes new command velocities until the goal is reached,
cancelled or an error occurs. It computes command velocities by using the
services of a controller plugin. The parameters in this section are set on the
controller_server node

Parameters
**********

:controller_frequency:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | double                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  20.0                                                                      |
  +----------------+----------------------------------------------------------------------------+
  |**units**       |  Hz                                                                        |
  +----------------+----------------------------------------------------------------------------+
  |**description** |  This parameter controls how often a new command velocity is               |
  |                |  published. It also controls how often the controller checks if the goal is|
  |                |  reached. Faster robots may need to increase this value. Slower robots can |
  |                |  use a lower value.                                                        |
  +----------------+----------------------------------------------------------------------------+

:controller_plugin_ids:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | vector<string>                                                             |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  "FollowPath"                                                              |
  +----------------+----------------------------------------------------------------------------+
  |**description** | The name of each controller plugin to be loaded into the controller server.|
  |                | Multiple controller plugins can be loaded simultaneously so they can share |
  |                | a costmap instance. The name assigned to the plugin using this parameter   |
  |                | determines how it should be addressed in the *controller_id* field of the  |
  |                | FollowPath action request.                                                 |
  +----------------+----------------------------------------------------------------------------+


:controller_plugin_types:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | vector<string>                                                             |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  "dwb_core::DWBLocalPlanner"                                               |
  +----------------+----------------------------------------------------------------------------+
  |**description** | The data type of each controller plugin to be loaded into the controller   |
  |                | server. This is used by pluginlib to dyanmically load the object. This     |
  |                | needs to be a class derived from nav2_core::LocalPlanner. The default      |
  |                | controller is an instance of DWB whose parameters are here                 |
  |                | `DWB Controller`_                                                          |
  +----------------+----------------------------------------------------------------------------+

Progress Checker
=================

This is a component of the controller server that verifies that the robot has moved sufficiently.
If the robot fails to make progress, the controller fails and triggers recoveries in the
behavior tree. The parameters in this section are set on the controller_server node.

Parameters
**********

:required_movement_radius:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | double                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  0.5                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**units**       |  m                                                                         |
  +----------------+----------------------------------------------------------------------------+
  |**description** | The absolute distance the robot must move from the previous checkpoint. If |
  |                | the robot doesn't move this far in *movement_time_allowance* seconds, an   |
  |                | error is thrown. Once the robot moves this distance the time is reset      |
  +----------------+----------------------------------------------------------------------------+

:movement_time_allowance:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | double                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  10.0                                                                      |
  +----------------+----------------------------------------------------------------------------+
  |**units**       |  s                                                                         |
  +----------------+----------------------------------------------------------------------------+
  |**description** | Controls how much time the robot has time to show progress. If the robot   |
  |                | fails to move more than *required_movement_radius* meters in this time, an |
  |                | error is thrown. For slow robots, this time should be extended. Reducing   |
  |                | this time will cause the robot to recover more quickly if it is stuck,     |
  |                | but too little time doesn't give the robot enough time to manoeuver in     |
  |                | dyanamic enviroments                                                       |
  +----------------+----------------------------------------------------------------------------+

DWB Controller
==============

The DWB controller is the default controller in |PN|. It is a fork of `David Lu's
controller <https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner>`_
modified for ROS 2.

.. warning::

  The parameters below are not properly scoped. The exact parameter names are
  likely to change in the near future

Parameters
**********

:trajectory_generator_name:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | string                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  "dwb_plugins::StandardTrajectoryGenerator"                                |
  +----------------+----------------------------------------------------------------------------+
  |**description** | This is the name of the pluginlib class to use as the trajectory generator.|
  |                | The trajectory generator projects a sample of possible trajectories the    |
  |                | robot can follow from its current pose and velocity. These trajectories    |
  |                | are then provided to the critics to score.                                 |
  |                |                                                                            |
  |                | You can create a custom trajectory generator by inheriting from the        |
  |                | dwb_core::TrajectoryGenerator base class                                   |
  |                |                                                                            |
  |                | The trajectory generators provided with |PN| are:                          |
  |                |                                                                            |
  |                | * dwb_plugins::StandardTrajectoryGenerator                                 |
  |                | * dwb_plugins::LimitedAccelGenerator                                       |
  +----------------+----------------------------------------------------------------------------+

:critics:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | vector<string>                                                             |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign",          |
  |                | "PathAlign", "PathDist", "GoalDist"]                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | This is a list of critic plugins that will evaluate the trajectories       |
  |                | generated by the trajectory generator. Each critic evaluates the           |
  |                | trajectories according to its own criteria. The scores from each critic    |
  |                | will be combined, and the trajectory with the best score will be chosen. By|
  |                | weighting the different critics appropriately, the quality of the chosen   |
  |                | trajectory can be modified.                                                |
  |                |                                                                            |
  |                | You can create a custom critic by inheriting from the                      |
  |                | dwb_core::TrajectoryCritic base class                                      |
  |                |                                                                            |
  |                | The critics provided with |PN| are:                                        |
  |                |                                                                            |
  |                | * dwb_critics::PreferForwardCritic                                         |
  |                | * dwb_critics::GoalDistCritic                                              |
  |                | * dwb_critics::PathAlignCritic                                             |
  |                | * dwb_critics::GoalAlignCritic                                             |
  |                | * dwb_critics::PathDistCritic                                              |
  |                | * dwb_critics::OscillationCritic                                           |
  |                | * dwb_critics::RotateToGoalCritic                                          |
  |                | * dwb_critics::BaseObstacleCritic                                          |
  |                | * dwb_critics::ObstacleFootprintCritic                                     |
  |                | * dwb_critics::TwirlingCritic                                              |
  +----------------+----------------------------------------------------------------------------+

:goal_checker_name:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | string                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  "dwb_plugins::SimpleGoalChecker"                                          |
  +----------------+----------------------------------------------------------------------------+
  |**description** | This is the plugin that evaluates the current pose and velocity to         |
  |                | determine if the goal has been reached                                     |
  |                |                                                                            |
  |                | You can create a custom goal checker by inheriting from the                |
  |                | nav2_core::GoalChecker base class                                          |
  |                |                                                                            |
  |                | The critics provided with |PN| are:                                        |
  |                |                                                                            |
  |                | * dwb_plugins::SimpleGoalChecker                                           |
  |                | * dwb_plugins::StoppedGoalChecker                                          |
  +----------------+----------------------------------------------------------------------------+

.. rst-class:: content-collapse

Additional Parameters
*********************

:prune_plan:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | true                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true, the path evaluated by the controller is trimmed to only include   |
  |                | points within *prune_distance* of the current robot pose. This improves CPU|
  |                | performance, and also has the effect of making the Goal* critics aim for a |
  |                | more nearby goal                                                           |
  +----------------+----------------------------------------------------------------------------+

:prune_distance:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | double                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | 1.0                                                                        |
  +----------------+----------------------------------------------------------------------------+
  |**units**       |  m                                                                         |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If *prune_plan* is true, all points in the path that are further away from |
  |                | the robot than this distance are discarded.                                |
  |                | This is re-evaluated each time the controller is run, so the points are not|
  |                | permanently discared.                                                      |
  +----------------+----------------------------------------------------------------------------+

:transform_tolerance:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | double                                                                     |
  +----------------+----------------------------------------------------------------------------+
  |**default**     |  0.1                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**units**       |  s                                                                         |
  +----------------+----------------------------------------------------------------------------+
  |**description** | When determining the current robot pose, if the transform data is older    |
  |                | than this, an error is returned                                            |
  +----------------+----------------------------------------------------------------------------+

Debug Parameters
================

:debug_trajectory_details:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | false                                                                      |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true and if all trajectories are rejected by the critics, DWB will print|
  |                | statistics on the console indicating which critics rejected the            |
  |                | trajectories and for what reason.                                          |
  +----------------+----------------------------------------------------------------------------+

:publish_cost_grid_pc:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | false                                                                      |
  +----------------+----------------------------------------------------------------------------+
  |**description** | Certain trajectory critics compute scores based on location in the costmap.|
  |                | For those critics, if this parameter is true, DWB will publish a PointCloud|
  |                | on the **cost_cloud** topic showing that critic's score for each cell. This|
  |                | can be visualized in RViz.                                                 |
  |                |                                                                            |
  |                | The standard critics provided with |PN| that provide this capability are:  |
  |                |                                                                            |
  |                | * dwb_critics::GoalDistCritic                                              |
  |                | * dwb_critics::PathAlignCritic                                             |
  |                | * dwb_critics::GoalAlignCritic                                             |
  |                | * dwb_critics::PathDistCritic                                              |
  |                | * dwb_critics::BaseObstacleCritic                                          |
  |                | * dwb_critics::ObstacleFootprintCritic                                     |
  +----------------+----------------------------------------------------------------------------+

:publish_evaluation:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | true                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true, publishes all evaluated trajectories and the scores each critic   |
  |                | assigned them. This is the most detailed information on what is going on in|
  |                | DWB, but can be hard to visualize. The data is published on the            |
  |                | **evaluation** topic                                                       |
  +----------------+----------------------------------------------------------------------------+

:publish_global_plan:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | true                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true, publishes the path recieved by the controller. This is published  |
  |                | on the **received_global_plan** topic and can be visualized in RViz        |
  +----------------+----------------------------------------------------------------------------+


:publish_local_plan:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | true                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true, published the chosen trajectory on the **local_plan** topic. This |
  |                | can be visualized in RViz.                                                 |
  +----------------+----------------------------------------------------------------------------+


:publish_trajectories:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | true                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true, all evaluated trajectories are published on the **marker** topic. |
  |                | The trajectories can be visualized in RViz and are color coded. Black means|
  |                | a critic rejected that particular trajectory as invalid. The remaining     |
  |                | trajectories are scaled from red to green, with the best trajectories being|
  |                | the most green and the worst being the most red.                           |
  +----------------+----------------------------------------------------------------------------+

  |                |                                                                            |

:publish_transformed_plan:
  +----------------+----------------------------------------------------------------------------+
  |**type**        | bool                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**default**     | true                                                                       |
  +----------------+----------------------------------------------------------------------------+
  |**description** | If true, publishes the received path after it has been transformed into the|
  |                | local frame of reference. This is published on the                         |
  |                | **transformed_global_plan** topic and can be visualized in RViz            |
  +----------------+----------------------------------------------------------------------------+


DWB Plugins
===========

.. toctree::
   :maxdepth: 1

   dwb-plugins/trajectory-generators.rst
   dwb-plugins/trajectory-critics.rst
   dwb-plugins/goal-checkers.rst
