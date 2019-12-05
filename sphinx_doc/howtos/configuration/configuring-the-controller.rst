.. _configuring_the_controller:

Configuring The Controller
##########################

Controller Server
=================

The controller server responds to FollowPath action requests. It runs a loop
that continuously computes new command velocities until the goal is reached,
cancelled or an error occurs. It computes command velocities by using the
services of a controller plugin. These parameters are set on the
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
  |                |  computed. It also controls how often the controller checks if the goal is |
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

This is a component of the controller server that ensures the robot is making progress towards
the goal. If it fails to make progress, the controller fails and triggers recoveries in the
behavior tree. These parameters are set on the controller_server node as well.

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
  |**description** | Controls how long the robot has time to show forward progress. If the robot|
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


BaseObstacle.class
BaseObstacle.scale
BaseObstacle.sum_scores
GoalAlign.aggregation_type
GoalAlign.class
GoalAlign.forward_point_distance
GoalAlign.scale
GoalDist.aggregation_type
GoalDist.class
GoalDist.scale
Oscillation.class
Oscillation.oscillation_reset_angle
Oscillation.oscillation_reset_dist
Oscillation.oscillation_reset_time
Oscillation.scale
Oscillation.x_only_threshold
PathAlign.aggregation_type
PathAlign.class
PathAlign.forward_point_distance
PathAlign.scale
PathDist.aggregation_type
PathDist.class
PathDist.scale
RotateToGoal.class
RotateToGoal.scale
RotateToGoal.xy_goal_tolerance
acc_lim_theta
acc_lim_x
acc_lim_y
controller_frequency
controller_plugin_ids
controller_plugin_types
critics
debug_trajectory_details
decel_lim_theta
decel_lim_x
decel_lim_y
discretize_by_time
goal_checker_name
max_speed_xy
max_vel_theta
max_vel_x
max_vel_y
min_speed_theta
min_speed_xy
min_theta_velocity_threshold
min_vel_x
min_vel_y
min_x_velocity_threshold
min_y_velocity_threshold
prune_distance
prune_plan
publish_cost_grid_pc
publish_evaluation
publish_global_plan
publish_local_plan
publish_trajectories
publish_transformed_plan
sim_time
trajectory_generator_name
transform_tolerance
use_dwa
use_sim_time
vx_samples
vy_samples
xy_goal_tolerance
yaw_goal_tolerance
