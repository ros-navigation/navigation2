# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors A plugin interface allows for the layers to be combined into the
costmap and finally inflated via a user specified inflation radius. The nav2 version of the costmap_2d package is mostly a direct
ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

## Overview of Changes from ROS1 Navigation Costmap_2d
- Removal of legacy parameter style ("Loading from pre-hydro parameter style")
- Intermediate replacement of dynamic reconfigure (not ported to ROS2). This discussion started here with costmap_2d but is a more
widespread discussion throughout the navigation stack (see issue https://github.com/ros-planning/navigation2/issues/177) and 
general ROS2 community. A proposal temporary replacement has been submitted as a PR here: https://github.com/ros-planning/navigation2/pull/196

## To use Voxel Layer Plugin:
Voxel layer allows for detecting obstacles in 3D using Pointcloud2 data. It requires Pointcloud2 data being published on some topic. In simulation, a Gazebo model of the robot with depth camera enabled will publish a pointcloud topic.

The Voxel Layer plugin can be used to update the local costmap, glocal costmap or both, depending on how you define it in the `nav2_params.yaml` file in the nav2_bringup directory. The voxel layer plugin has to be added to the list of ```plugin_names``` and ```plugin_types``` in the global/local costmap scopes in the param file. The `observation_sources` in the local/global costmap scope in the param file should also have `pointcloud` as one of it's sources.

Example param configuration for voxel layer enabled in local costmap is shown below:
```
local_costmap:
  local_costmap:
    ros__parameters:
      plugin_names: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      plugin_types: ["nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::VoxelLayer", "nav2_costmap_2d::InflationLayer"]
      rolling_window: true
      inflation_layer:
        cost_scaling_factor: 3.0
      obstacle_layer:
        enabled: True
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          data_type: "LaserScan"
      voxel_layer:
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        pointcloud:
          topic: /intel_realsense_r200_depth/points
            max_obstacle_height: 2.0
          data_type: "PointCloud2"
      static_layer:
        map_subscribe_transient_local: True
      observation_sources: scan pointcloud
```


To visualize the voxels in RVIZ, open up a new terminal and run:
- ```ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker```

- Add `my_marker` to RVIZ using the GUI.

Errata:
- Currently due to some bug in rviz, you need to change set the `fixed_frame` in the rviz display, to `odom` frame.
- Using pointcloud data from bag file, while using gazebo simulation can be troublesome due to the clock time skipping to an earlier time.

## Future Plans
- Conceptually, the costmap_2d model acts as a world model of what is known from the map, sensor, robot pose, etc. We'd like
to broaden this world model concept and use costmap's layer concept as motivation for providing a service-style interface to
potential clients needing information about the world (see issue https://github.com/ros-planning/navigation2/issues/18)
