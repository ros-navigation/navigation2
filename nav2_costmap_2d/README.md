# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors A plugin interface allows for the layers to be combined into the
costmap and finally inflated via a user specified inflation radius. The nav2 version of the costmap_2d package is mostly a direct
ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

## Overview of Changes from ROS1 Navigation Costmap_2d
- Removal of legacy parameter style ("Loading from pre-hydro parameter style")
- Intermediate replacement of dynamic reconfigure (not ported to ROS2). This discussion started here with costmap_2d but is a more
widespread discussion throughout the navigation stack (see issue https://github.com/ros-planning/navigation2/issues/177) and 
general ROS2 community. A proposal temporary replacement has been submitted as a PR here: https://github.com/ros-planning/navigation2/pull/196

## How to configure using Voxel Layer plugin:
By default, the navigation stack uses the _Obstacle Layer_ to avoid obstacles in 2D. The _Voxel Layer_ allows for detecting obstacles in 3D using Pointcloud2 data. It requires Pointcloud2 data being published on some topic. For simulation, a Gazebo model of the robot with depth camera enabled will publish a pointcloud topic.

The Voxel Layer plugin can be used to update the local costmap, glocal costmap or both, depending on how you define it in the `nav2_params.yaml` file in the nav2_bringup directory. The voxel layer plugin has to be added to the list of ```plugins``` and its ```plugin``` type should be correctly specified in the global/local costmap scopes in the param file. If these are not defined in the param file, the default plugins set in nav2_costmap_2d will be used.

Inside each costmap layer (voxel, obstacle, etc) define `observation_sources` param. Here you can define multiple sources to be used with the layer. The param configuration example below shows the way you can configure costmaps to use voxel layer.

The `voxel_layer` param has `observation_source: pointcloud` in it's scope, and the param `pointcloud` has `topic`, and `data_type` inside it's scope. By default, the data_type is `LaserScan`, thus you need to specify `PointCloud2` if you are using PointCould2 data from a depth sensor.

Example param configuration snippet for enabling voxel layer in local costmap is shown below (not all params are shown):
```
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      voxel_layer:
        plugin: nav2_costmap_2d::VoxelLayer
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /intel_realsense_r200_depth/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
```
Please note that not all params needed to run the navigation stack are shown here. This example only shows how you can add different costmap layers, with multiple input sources of different types.

### To visualize the voxels in RVIZ:
- Make sure `publish_voxel_map` in `voxel_layer` param's scope is set to `True`.
- Open a new terminal and run:
  ```ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker```
    Here you can change `my_marker` to any topic name you like for the markers to be published on.

- Then add `my_marker` to RVIZ using the GUI.


#### Errata:
- To see the markers in 3D, you will need to change the _view_ in RVIZ to a 3 dimensional view (e.g. orbit) from the RVIZ GUI.
- Currently due to some bug in rviz, you need to set the `fixed_frame` in the rviz display, to `odom` frame.
- Using pointcloud data from a saved bag file while using gazebo simulation can be troublesome due to the clock time skipping to an earlier time.

## How to use multiple sensor sources:
Multiple sources can be added into a costmap layer (e.g., obstacle layer), by listing them under the 'observation_sources' for that layer.
For example, to add laser scan and pointcloud as two different sources of inputs to the obstacle layer, you can define them in the params file as shown below for the local costmap:
```
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan pointcloud
        scan:
          topic: /scan
          data_type: "LaserScan"
        pointcloud:
          topic: /intel_realsense_r200_depth/points
          data_type: "PointCloud2"
```
In order to add multiple sources to the global costmap, follow the same procedure shown in the example above, but now adding the sources and their specific params under the `global_costmap` scope.

## Costmap Filters

### Overview

Costmap Filters - is a costmap layer-based instrument which provides an ability to apply to map spatial-dependent raster features named as filter-masks. These features are used in plugin algorithms when filling costmaps in order to allow robots to change their trajectory, behavior or speed when a robot enters/leaves an area marked in a filter masks. Examples of costmap filters include keep-out/safety zones where robots will never enter, speed restriction areas, preferred lanes for robots moving in industries and warehouses. More information about design, architecture of the feature and how it works could be found on Nav2 website: https://navigation.ros.org.

## Future Plans
- Conceptually, the costmap_2d model acts as a world model of what is known from the map, sensor, robot pose, etc. We'd like
to broaden this world model concept and use costmap's layer concept as motivation for providing a service-style interface to
potential clients needing information about the world (see issue https://github.com/ros-planning/navigation2/issues/18)

