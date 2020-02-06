.. _navigation2-on-real-turtlebot3:

Navigation2 With External Costmap Plugin
****************************************

- `Overview`_
- `Costmap2D and STVL`_
- `Tutorial Steps`_
- `Video`_

Overview
========

This tutorial shows how to load and use an external plugin.
This example uses the `Spatio Temporal Voxel Layer <https://github.com/SteveMacenski/spatio_temporal_voxel_layer/>`_ (STVL) costmap `pluginlib <http://wiki.ros.org/pluginlib/>`_ plugin as an example.
STVL is a demonstrative pluginlib plugin and the same process can be followed for other costmap plugins as well as plugin planners, controllers, and recoveries.

Before completing this tutorial, please look at the previous two tutorials on navigation in simulation and physical hardware, if available.
This tutorial assumes knowledge of navigation and basic understanding of costmaps.

Costmap2D and STVL
==================

Costmap 2D is the data object we use to buffer sensor information into a global view that the robot will use to create plans and control efforts.
Within Costmap2D, there are pluginlib plugin interfaces available to create custom behaviors loadable at runtime.
Examples of included pluginlib plugins for Costmap2D are the Obstacle Layer, Voxel Layer, Static Layer, and Inflation Layer.

However, these are simply example plugins offered by the base implementation.
Another available pluginlib plugin for Costmap2D in Navigation2 is STVL.

STVL is another 3D perception plugin similar to the Voxel Layer.
A more detailed overview of how it works can be found `here <https://github.com/SteveMacenski/spatio_temporal_voxel_layer/>`_, however it buffers 3D data from depth cameras, sonars, lidars, and more into a sparse volumetic world model and removes voxels over time proportional with a sensor model and time-based expiration.
This can be especially useful for robots in highly dynamic envrionments and decreases the resource utilization for 3D sensor processing by up to 2x.
STVL also treats 3D lidars and radars as first class citizens for support.

Tutorial Steps
==============

0- Setup
--------

Follow the same process as in :ref:`navigation2-with-turtlebot3-in-gazebo` for installing and setting up a robot for hardware testing or simulation, as applicable. Ensure ROS2, Navigation2, and Gazebo are installed.

1- Install STVL
---------------

STVL can be installed in ROS2 Dashing and Eloquent via the ROS Build Farm:

- ``sudo apt install ros-<ros2-distro>-spatio-temporal-voxel-layer``

It can also be built from source by cloning the repository into your Navigation2 workspace:

- ``git clone -b <ros2-distro>-devel git@github.com:stevemacenski/spatio_temporal_voxel_layer``

1- Modify Navigation2 Parameter
-------------------------------

STVL is an optional plugin, like all plugins, in Costmap2D. Costmap Plugins in Navigation2 are loaded in the ``plugin_names`` and ``plugin_types`` variables inside of their respective costmaps.
For example, the following will load the static and obstacle layer plugins into the name ``static_layer`` and ``obstacle_layer``, respectively:

.. code-block:: yaml

    global_costmap:
      global_costmap:
        ros__parameters:
          use_sim_time: True
          plugin_names: ["static_layer", "obstacle_layer"]
          plugin_types: ["nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer"]

To load the STVL plugin, a new plugin name and type must be added.
For example, if the application required an STVL layer and no obstacle layer, our file would be:

.. code-block:: yaml

    global_costmap:
      global_costmap:
        ros__parameters:
          use_sim_time: True
          plugin_names: ["static_layer", "stvl_layer"]
          plugin_types: ["nav2_costmap_2d::StaticLayer", "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"]

Similar to the Voxel Layer, after registering the plugin, we can add the configuration of the STVL layer under the namespace ``stvl_layer``.
An example fully-described parameterization of an STVL configuration is:

.. code-block:: yaml

    stvl_layer:
      enabled: true
      voxel_decay: 15.
      decay_model: 0
      voxel_size: 0.05
      track_unknown_space: true
      max_obstacle_height: 2.0
      unknown_threshold: 15
      mark_threshold: 0
      update_footprint_enabled: true
      combination_method: 1
      obstacle_range: 3.0
      origin_z: 0.0
      publish_voxel_map: true
      transform_tolerance: 0.2
      mapping_mode: false
      map_save_duration: 60.0
      observation_sources: pointcloud
      pointcloud:
        data_type: PointCloud2
        topic: /intel_realsense_r200_depth/points
        marking: true
        clearing: true
        min_obstacle_height: 0.0
        max_obstacle_height: 2.0
        expected_update_rate: 0.0
        observation_persistence: 0.0
        inf_is_valid: false
        voxel_filter: false
        clear_after_reading: true
        max_z: 7.0
        min_z: 0.1
        vertical_fov_angle: 0.8745
        horizontal_fov_angle: 1.048
        decay_acceleration: 15.0
        model_type: 0

Please copy-paste the text above, with the ``plugin_names`` and ``plugin_types`` registration, into your ``nav2_params.yaml`` to enable STVL in your application.
Make sure to change both the local and global costmaps.

Note: Pluginlib plugins for other Navigation2 servers such as planning, recovery, and control can be set in this same way.

2- Launch Navigation2
---------------------

Follow the same process as in :ref:`navigation2-with-turtlebot3-in-gazebo` to launch a simulated robot in gazebo with Navigation2.
Navigation2 is now using STVL as its 3D sensing costmap layer.

3-  RVIZ
--------

With RViz open and ``publish_voxel_map: true``, you can visualize the underlying data structure's 3D grid using the ``{local, global}_costmap/voxel_grid`` topics.
Note: It is recommended in RViz to set the ``PointCloud2`` Size to your voxel size and the style to ``Boxes`` with a neutral color for best visualization.

Video
-----

.. raw:: html

    <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
      <iframe width="960" height="720" src="https://www.youtube.com/embed/TGxb1OzgmNQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    </div>
