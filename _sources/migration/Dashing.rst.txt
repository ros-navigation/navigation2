.. _dashing_migration:

Dashing to Eloquent
###################

Moving from ROS2 Dashing to Eloquent, a number of stability improvements were added that we will not specifically address here.

New Packages
************

Navigation2 now includes a new package ``nav2_waypoint_follower``.
The waypoint follower is an action server that will take in a list of waypoints to follow and follow them in order.
There is a parameter ``stop_on_failure`` whether the robot should continue to the next waypoint on a single waypoint failure,
or to return fail to the action client.
The waypoint follower is also a reference application for how to use the Navigation2 action server to complete a basic autonomy task.


Navigation2 now supports new algorithms for control and SLAM.
The Timed-Elastic Band (TEB) controller was implemented `and can be found here <https://github.com/rst-tu-dortmund/teb_local_planner>`_.
It is its own controller plugin that can be used instead of the DWB controller.
Nav2 also supports SLAM Toolbox as the default SLAM implementation for ROS2.
This replaces the use of Cartographer.

New Plugins
***********

Eloquent introduces back in pluginlib plugins to the navigation stack.
``nav2_core`` defines the plugin header interfaces to be used to implement controller, planner, recovery, and goal checker plugins.
All algorithms (NavFn, DWB, recoveries) were added as plugin interfaces and the general packages for servers were created.
``nav2_planner`` is the action server for planning that hosts a plugin for the planner.
``nav2_controller`` is the action server for controller that hosts a plugin for the controller.
``nav2_recovery`` is the action server for recovery that hosts a plugin for recovery.

New recovery plugins were added including backup, which will take in a distance to back up, if collision-free.
Additionally, the wait recovery was added that will wait a configurable period of time before trying to navigate again.
This plugin is especially helpful for time-dependent obstacles or pausing navigation for a scene to become less dynamic.

Many new behavior tree nodes were added. These behavior tree nodes are hard-coded in the behavior tree engine.
Behavior tree cpp v3 supports plugins and will be converted in the next release.

Navigation2 Architectural Changes
*********************************

The ``nav2_world_model`` package was removed. The individual ``nav2_planner`` and ``nav2_controller`` servers now host their relevent costmaps.
This was done to reduce network traffic and ensure up-to-date information for the safety-critical elements of the system.
As above mentions, plugins were introduced into the stack and these servers each host plugins for navigation, control, and costmap layers.

Map server was substantially refactored but the external API remains the same. It now uses the SDL library for image loading.

TF-based positioning is now used for pose-estimation everywhere in the stack.
Prior, some elements of the navigation stack only updated its pose from the ``/amcl_pose`` topic publishing at an irregular rate.
This is obviously low-accuracy and high-latency.
All positioning is now based on the TF tree from the global frame to the robot frame.

Prior to Eloquent, there were no ROS2 action servers and clients available.
Navigation2, rather, used an interface we called Tasks.
Eloquent now contains actions and a simple action server interface was created and is used now throughout the stack.
Tasks were removed.
