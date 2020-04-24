.. _eloquent_migration:

Eloquent to Foxy
################

Moving from ROS2 Eloquent to Foxy, a number of stability improvements were added that we will not specifically address here.
We will specifically mention, however, the reduction in terminal noise.
TF2 transformation timeout errors and warnings on startup have been largely removed or throttled to be more tractable.
Additionally, message filters filling up resulting in messages being dropped were resolved in costmap 2d.

General
*******

The lifecycle manager was split into 2 unique lifecycle managers.
They are the ``navigation_lifecycle_manager`` and ``localization_lifecycle_manager``.
This gives each process their own manager to allow users to switch between SLAM and localization without effecting Navigation.
It also reduces the redundant code in ``nav2_bringup``.

A fix to the BT navigator was added to remove a rare issue where it may crash due to asynchronous issues.
As a result, a behavior tree is created for each navigation request rather than resetting an existing tree.
The creation of this tree will add a small amount of latency.
Proposals to reduce this latency will be considered before the next release.

Server Updates
**************
All plugin servers (controller, planner, recovery) now supports the use of multiple plugins.
This can be done by loading a map of plugins, mapping the name of the plugin to its intended use-case.
An example: ``FollowPath`` controller of type ``dwb_local_planner/DWBLocalPlanner`` and a ``DynamicFollowPath`` of type ``teb_local_planner/TEBLocalPlanner``.
Each plugin will load the parameters in their namespace, e.g. ``FollowPath.max_vel_x``, rather than globally in the server namespace.
This will allow multiple plugins of the same type with different parameters and reduce conflicting parameter names.

DWB Contains new parameters as an update relative to the ROS1 updates, `see here for more information <https://github.com/ros-planning/navigation2/pull/1501>`_.
Additionally, the controller and planner interfaces were updated to include a ``std::string name`` parameter on initialization.
This was added to the interfaces to allow the plugins to know the namespace it should load its parameters in.
E.g. for a controller to find the parameter ``FollowPath.max_vel_x``, it must be given its name, ``FollowPath`` to get this parameter.
All plugins will be expected to look up parameters in the namespace of its given name. 

New Plugins
***********

Many new behavior tree nodes were added.
These behavior tree nodes are now BT plugins and dynamically loadable at run-time using behavior tree cpp v3.
See ``nav2_behavior_tree`` for a full listing, or :ref:`plugins` for the current list of behavior tree plugins and their descriptions. 
These plugins are set as default in the ``nav2_bt_navigator`` but may be overrided by the ``bt_plugins`` parameter to include your specific plugins.

Map Server Re-Work
******************

``map_saver`` was re-worked and divided into 2 parts: CLI and server.
CLI part is a command-line tool that listens incoming map topic, saves map once into a file and finishes its work. This part is remained to be almost untouched: CLI executable was renamed from ``map_saver`` to ``map_saver_cli`` without changing its functionality.
Server is a new part. It spins in the background and can be used to save map continuously through a ``save_map`` service. By each service request it tries to listen incoming map topic, receive a message from it and write obtained map into a file.

``map_server`` was dramatically simplified and cleaned-up. ``OccGridLoader`` was merged with ``MapServer`` class as it is intended to work only with one ``OccupancyGrid`` type of messages in foreseeable future.

``map_loader`` was completely removed from ``nav2_util``. All its functionality already present in ``map_io`` library. Please use it in your code instead.

Please refer to the `original GitHub ticket <https://github.com/ros-planning/navigation2/issues/1010>`_ and `Map Server README <https://github.com/ros-planning/navigation2/blob/master/nav2_map_server/README.md>`_ for more information.
