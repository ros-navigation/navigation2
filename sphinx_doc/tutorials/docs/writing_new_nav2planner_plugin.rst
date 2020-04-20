.. _writing_new_nav2planner_plugin:

Writing a New Planner Plugin
****************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

.. image:: images/Writing_new_nav2planner_plugin/nav2_straightline_gif.gif
    :width: 700px
    :align: center
    :alt: Animated gif with gradient demo

Overview
========

This tutorial shows how to create you own planner plugin.

Requirements
============

- ROS2 (binary or build-from-source)
- Navigation2 (Including dependencies)
- Gazebo
- Turtlebot3

Tutorial Steps
==============

1- Creating a new Planner Plugin
--------------------------------

We will create a simple straight line planner.
The annotated code in this tutorial can be found in `navigation_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ repository as the ``nav2_straightline_planner``
This package can be a considered as a reference for writing planner plugin.

Our example plugin inherits from the base class ``nav2_core::GlobalPlanner``. The base class provides 5 pure virtual methods to implement a planner plugin. The plugin will be used by the planner server to compute trajectories.
Lets learn more about the methods needed to write a planner plugin.

+----------------------+----------------------------------------------------------------------------+-------------------------+
| **Virtual method**   | **Method description**                                                     | **Requires override?**  |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| configure()          | Method is called at when planner server enters on_configure state. Ideally | Yes                     |
|                      | this methods should perform declarations of ROS parameters and             |                         |
|                      | initialization of planner's member variables. This method takes 4 input    |                         |
|                      | params: shared pointer to parent node, planner name, tf buffer pointer and |                         |
|                      | shared pointer to costmap.                                                 |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| activate()           | Method is called when planner server enters on_activate state. Ideally this| Yes                     |
|                      | method should implement operations which are neccessary before planner goes|                         |
|                      | to an active state.                                                        |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| deactivate()         | Method is called when planner server enters on_deactivate state. Ideally   | Yes                     |
|                      | this method should implement operations which are neccessary before planner|                         |
|                      | goes to an inactive state.                                                 |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| cleanup()            | Method is called when planner server goes to on_cleanup state. Ideally this| Yes                     |
|                      | method should clean up resoures which are created for the planner.         |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+
| createPlan()         | Method is called when planner server demands a global plan for specified   | Yes                     |
|                      | start and goal pose. This method returns `nav_msgs::msg::Path` carrying    |                         |
|                      | global plan. This method takes 2 input parmas: start pose and goal pose.   |                         |
+----------------------+----------------------------------------------------------------------------+-------------------------+

For this tutorial, we will be using methods ``StraightLine::configure()`` and ``StraightLine::createPlan()`` to create straight line planner.

In planners, ``configure()`` method must set member variables from ROS parameters and any initialization required,

.. code-block:: c++

  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

Here, ``name_ + ".interpolation_resolution"`` is fetching the ROS parameters ``interpolation_resolution`` which is specific to our planner. Navigation2 allows loading of multiple plugins and to keep things organized each plugin is mapped to some ID/name. Now if we want to retrieve the parameters for that specific plugin, we use ``<mapped_name_of_plugin>.<name_of_parameter>`` as done in the above snippet. For example, our example planner is mapped to the name "GridBased" and to retrieve the ``interpolation_resolution`` parameter which is specific to "GridBased", we used ``Gridbased.interpolation_resolution``. In other words, ``GridBased`` is used as a namespace for plugin-specific parameters. We will see more on this when we discuss the parameters file (or params file).

In ``createPlan()`` method, we need to create a path from the given start to goal poses. The ``StraightLine::createPlan()`` is called using start pose and goal pose to solve the global path planning problem. Upon succeeding, it converts the path to the ``nav_msgs::msg::Path`` and returns to the planner server. Below annotation shows the implementation of this method.

.. code-block:: c++

  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  global_path.poses.push_back(goal);

  return global_path;

The remaining methods are not used but its mandatory to override them. As per the rules, we did override all but left them blank.

2- Exporting the planner plugin
-------------------------------

Now that we have created our custom planner, we need to export our planner plugin so that it would be visible to the planner server. Plugins are loaded at runtime and if they are not visible, then our planner server won't be able to load it. In ROS2, exporting and loading plugins is handled by ``pluginlib``.

Coming to our tutorial, class ``nav2_straightline_planner::StraightLine`` is loaded dynamically as ``nav2_core::GlobalPlanner`` which is our base class.

1. To export the planner, we need to provide two lines

.. code-block:: c++
  
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)

Note that it requires pluginlib to export out plugin's class. Pluginlib would provide as macro ``PLUGINLIB_EXPORT_CLASS`` which does all the work of exporting.

It is good practice to place these lines at the end of the file but technically, you can also write at the top.

2. Next step would be to create plugin's description file in the root directory of the package. For example, ``global_planner_plugin.xml`` file in our tutorial package. This file contains following information

 - ``library path``: Plugin's library name and it's location.
 - ``class name``: Name of the class.
 - ``class type``: Type of class.
 - ``base class``: Name of the base class.
 - ``description``: Description of the plugin.

.. code-block:: xml

  <library path="nav2_straightline_planner_plugin">
    <class name="nav2_straightline_planner/StraightLine" type="nav2_straightline_planner::StraightLine" base_class_type="nav2_core::GlobalPlanner">
      <description>This is an example plugin which produces straight path.</description>
    </class>
  </library>

3. Next step would be to export plugin using ``CMakeLists.txt`` by using cmake function ``pluginlib_export_plugin_description_file()``. This function installs plugin description file to ``share`` directory and sets ament indexes to make it discoverable.

.. code-block:: text

  pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

4. Plugin description file should also be added to ``package.xml``

.. code-block:: xml

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/global_planner_plugin.xml" />
  </export>

5. Compile and it should be registered. Next, we'll use this plugin.

3- Pass the plugin name through params file
-------------------------------------------

To enable the plugin, we need to modify the ``nav2_params.yaml`` file as below

replace following params

.. code-block:: text

  planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_navfn_planner/NavfnPlanner"]
    planner_plugin_ids: ["GridBased"]
    use_sim_time: True
    GridBased.tolerance: 2.0
    GridBased.use_astar: false
    GridBased.allow_unknown: true

with

.. code-block:: text

  planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_straightline_planner/StraightLine"]
    planner_plugin_ids: ["GridBased"]
    use_sim_time: True
    GridBased.interpolation_resolution: 0.1

In the above snippet, you can observe the mapping of our ``nav2_straightline_planner/StraightLine`` planner to its id ``GridBased``. To pass plugin-specific parameters we have used ``<plugin_id>.<plugin_specific_parameter>``.

4- Run StraightLine plugin
---------------------------

Run Turtlebot3 simulation with enabled navigation2. Detailed instruction how to make it are written at :ref:`getting_started`. Below is shortcut command for that:

.. code-block:: bash

  $ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml

Then goto RViz and click on the "2D Pose Estimate" button at the top and point the location on map as it was described in :ref:`getting_started`. Robot will localize on the map and then click on "Navigation2 goal" and click on the pose where you want your planner to consider a goal pose. After that planner will plan the path and robot will start moving towards the goal.
