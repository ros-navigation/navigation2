.. _writing_new_nav2planner_plugin:

Writing a new planner plugin
****************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

.. image:: images/Writing_new_nav2planner_plugin/nav2_rrtconnect_gif.gif
    :width: 700px
    :align: center
    :alt: Animated gif with rrtconnect demo

Overview
========

This tutorial shows how to create you own planner plugin.

Requirements
============

- ROS2 (binary or build-from-source)
- Navigation2 (Including dependencies)
- Gazebo
- Turtlebot3
- `OMPL (binary or build-from-source) <https://ompl.kavrakilab.org/installation.html>`_

Tutorial Steps
==============

1- Creating a new Planner Plugin
--------------------------------

We will create a Sampling base planner - RRT connect using OMPL library.
The annotated code in this tutorial can be found in `navigation_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ repository as the ``nav2_rrtconnect_plugin``
This package can be a considered as a reference for writing planner plugin.

So, the class ``nav2_rrtconnect_planner::RRTConnect`` example plugin inherites from base class ``nav2_core::GlobalPlanner``. The base class provides 5 pure virtual methods API to implement a planner plugin in Navigation 2. The implemented plugin will be used by the planner server which is of `LifecycleNode <https://design.ros2.org/articles/node_lifecycle.html>`_ type.
Lets see and learn more about these virtual methods which we have to override in order to write a planner plugin.

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

For this tutorial, we will be using methods ``RRTConnect::configure()`` and ``RRTConnect::createPlan()`` to create RRT Connect planner.

In our planner, ``RRTConnect::configure()`` method sets member variables and ROS parameters,

.. code-block:: c

  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".solve_time", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".solve_time", solve_time_);
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_trials", rclcpp::ParameterValue(4));
  node_->get_parameter(name_ + ".max_trials", max_trials_);

and also, it sets OMPL variables such as bounds, setup settings and space validation.

In ``RRTConnect::createPlan()`` method, RRT Connect planner is called using start pose and goal pose to solve the global path planning problem. Upon succeding, it converts the path to the ``nav_msgs::msg::Path`` and returns to the caller which is planner server instance.

The remaining methods are not used but its mandatory to override them. As per the rules, we did override all but left them blank.

2- Exporting the planner plugin
-------------------------------

Now that we have created our custom planner, we need to export our planner plugin so that it would be visible to the planner server. Plugins are loaded at runtime and if they are not visible, then our planner server won't be able to load it. In ROS2, exporting and loading plugins is handled by ``pluginlib``.

Coming to our tutorial, class ``nav2_rrtconnect_planner::RRTConnect`` is loaded dynamically as ``nav2_core::GlobalPlanner`` which is our base class.

1. To export the planner, we need to provide two lines

.. code-block:: c
  
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_rrtconnect_planner::RRTConnect, nav2_core::GlobalPlanner)

Note that it requires pluginlib to export out plugin's class. Pluginlib would provide as macro ``PLUGINLIB_EXPORT_CLASS`` which does all the work of exporting.

It is good practice to place this lines at the end of the file but in practice you can also write at the top.

2. Next step would be to create plugin's description file in the root directory of the package. For example, ``global_planner_plugin.xml`` file in our tutorial package. This file contains following information

 - ``library path``: Plugin's library name and it's location.
 - ``class name``: Name of the class.
 - ``class type``: Type of class.
 - ``base class``: Name of the base class.
 - ``description``: Description of the plugin.

.. code-block:: xml

  <library path="nav2_rrtconnect_planner_plugin">
    <class name="nav2_rrtconnect_planner/RRTConnect" type="nav2_rrtconnect_planner::RRTConnect" base_class_type="nav2_core::GlobalPlanner">
      <description>This is an example plugin which produces global path utilizing OMPL's implemented RRT Connect Algorithm.</description>
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

5. Put the package in a ROS2 workspace and compile. 

3- Pass the plugin name through params file
-------------------------------------------

To enable the plugin, we need to modify the ``nav2_params.yml`` file as below

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
    planner_plugin_types: ["nav2_rrtconnect_planner/RRTConnect"]
    use_sim_time: True
    GridBased.solve_time: 1.0
    GridBased.max_trials: 4
    GridBased.collision_checking_resolution: 0.001
    GridBased.allow_unknown: True

4- Run GradientLayer plugin
---------------------------

Run Turtlebot3 simulation with enabled navigation2. Detailed instruction how to make it are written at :ref:`getting_started`. Below is shortcut command for that:

.. code-block:: bash

  $ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml

Then goto RViz and click on the "2D Pose Estimate" button at the top and point the location on map as it was described in :ref:`getting_started`. Robot will localize on the map and then click on "Navigation2 goal" and click on the pose where you want your planner to consider a goal pose. After that planner will plan the path and robot will start moving towards the goal. Please checkout the video below for demo.

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/MSy7WAE_xz4?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>
