*NOTE: <> means plugin are namespaced by a name/id parameterization. The bracketed names may change due to your application configuration*

# bt_navigator 

| Parameter | Default | Description |
| ----------| --------| ------------|
| bt_xml_filename | N/A |  |
| plugin_lib_names | ["nav2_compute_path_to_pose_action_bt_node", "nav2_follow_path_action_bt_node", "nav2_back_up_action_bt_node", "nav2_spin_action_bt_node", "nav2_wait_action_bt_node", "nav2_clear_costmap_service_bt_node", "nav2_is_stuck_condition_bt_node", "nav2_goal_reached_condition_bt_node", "nav2_initial_pose_received_condition_bt_node", "nav2_goal_updated_condition_bt_node", "nav2_reinitialize_global_localization_service_bt_node", "nav2_rate_controller_bt_node", "nav2_distance_controller_bt_node", "nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node", "nav2_round_robin_node_bt_node", "nav2_transform_available_condition_bt_node"] |  |
| transform_tolerance | 0.1 |  |
| global_frame | "map" |  |
| robot_base_frame | "base_link" |  |

# costmaps

## Node: costmap_2d_ros 

Namespace: /parent_ns/local_ns

| Parameter | Default | Description |
| ----------| --------| ------------|
| always_send_full_costmap | false | |
| footprint_padding | 0.01 | |
| footprint | "[]" | |
| global_frame | "map" | |
| height | 5 | |
| width | 5 | |
| lethal_cost_threshold | 100 | |
| map_topic | "parent_namespace/map" | |
| observation_sources | "" | |
| origin_x | 0.0 | |
| origin_y | 0.0 | |
| plugin_names | {"static_layer", "obstacle_layer", "inflation_layer"} | |
| plugin_types | {"nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::InflationLayer"} | |
| publish_frequency | 1.0 | |
| resolution | 0.1 | |
| robot_base_frame | "base_link" | |
| robot_radius| 0.1 | |
| rolling_window | false | |
| track_unknown_space | false | |
| transform_tolerance | 0.3 | |
| trinary_costmap | true | |
| unknown_cost_value | 255 | |
| update_frequency | 5.0 | 
| use_maximum | false | |
| clearable_layers | "obstacle_layer" | |

### static_layer plugin

* `<static layer>`: Name corresponding to the `nav2_costmap_2d::StaticLayer` plugin. This name gets defined in `plugin_names`, default value is `static_layer`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<static layer>`.enabled | true | |
| `<static layer>`.subscribe_to_updates | false | |
| `<static layer>`.subscribe_to_updates | false | |
| `<static layer>`.map_subscribe_transient_local | true | |
| `<static layer>`.transform_tolerance | 0.0 | |

### inflation_layer plugin

* `<inflation layer>`: Name corresponding to the `nav2_costmap_2d::InflationLayer` plugin. This name gets defined in `plugin_names`, default value is `inflation_layer`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<inflation layer>`.enabled | true | |
| `<inflation layer>`.inflation_radius | 0.55 | |
| `<inflation layer>`.cost_scaling_factor | 10.0 | |
| `<inflation layer>`.inflate_unknown | false | |
| `<inflation layer>`.inflate_around_unknown | false | |

### obstacle_layer plugin

* `<obstacle layer>`: Name corresponding to the `nav2_costmap_2d::ObstacleLayer` plugin. This name gets defined in `plugin_names`, default value is `obstacle_layer`
* `<data source>`: Name of a source provided in ``<obstacle layer>`.observation_sources`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<obstacle layer>`.enabled | true | |
| `<obstacle layer>`.footprint_clearing_enabled | true | |
| `<obstacle layer>`.max_obstacle_height | 2.0 | |
| `<obstacle layer>`.combination_method | 1 | |
| `<obstacle layer>`.observation_sources | "" | data source topic |
| `<data source>`.topic  | "" | |
| `<data source>`.sensor_frame | "" | |
| `<data source>`.observation_persistence | 0.0 | |
| `<data source>`.expected_update_rate | 0.0 | |
| `<data source>`.data_type | "LaserScan" | |
| `<data source>`.min_obstacle_height | 0.0 | |
| `<data source>`.max_obstacle_height | 0.0 | |
| `<data source>`.inf_is_valid | false | |
| `<data source>`.marking | true | |
| `<data source>`.clearing | false | |
| `<data source>`.obstacle_range | 2.5 | |
| `<data source>`.raytrace_range | 3.0 | | 

### voxel_layer plugin

* `<voxel layer>`: Name corresponding to the `nav2_costmap_2d::VoxelLayer` plugin. This name gets defined in `plugin_names`
* `<data source>`: Name of a source provided in `<voxel layer>`.observation_sources`

*Note*: These parameters will only get declared if a `<voxel layer>` name such as `voxel_layer` is appended to `plugin_names` parameter and `"nav2_costmap_2d::VoxelLayer"` is appended to `plugin_types` parameter.

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<voxel layer>`.enabled | true | |
| `<voxel layer>`.footprint_clearing_enabled | true | |
| `<voxel layer>`.max_obstacle_height | 2.0 | |
| `<voxel layer>`.z_voxels | 10 | |
| `<voxel layer>`.origin_z | 0.0 | |
| `<voxel layer>`.z_resolution | 0.2 | |
| `<voxel layer>`.unknown_threshold | 15 | |
| `<voxel layer>`.mark_threshold | 0 | |
| `<voxel layer>`.combination_method | 1 | |
| `<voxel layer>`.publish_voxel_map | false | |
| `<voxel layer>`.observation_sources | "" | data source topic |
| `<data source>`.topic  | "" | |
| `<data source>`.sensor_frame | "" | |
| `<data source>`.observation_persistence | 0.0 | |
| `<data source>`.expected_update_rate | 0.0 | |
| `<data source>`.data_type | "LaserScan" | |
| `<data source>`.min_obstacle_height | 0.0 | |
| `<data source>`.max_obstacle_height | 0.0 | |
| `<data source>`.inf_is_valid | false | |
| `<data source>`.marking | true | |
| `<data source>`.clearing | false | |
| `<data source>`.obstacle_range | 2.5 | |
| `<data source>`.raytrace_range | 3.0 | | 

## controller_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| controller_frequency | 20.0 | |
| controller_plugin_ids | "FollowPath" | |
| controller_plugin_types | "dwb_core::DWBLocalPlanner" | |
| min_x_velocity_threshold | 0.0001 | |
| min_y_velocity_threshold | 0.0001 | |
| min_theta_velocity_threshold | 0.0001 | |
| required_movement_radius | 0.5 | |
| movement_time_allowance | 10.0 | |

# dwb_controller

* `<dwb plugin>`: DWB plugin name defined in `controller_plugin_ids` in the controller_server parameters

### dwb_local_planner

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.critics | | |
| `<dwb plugin>`.default_critic_namespaces | ["dwb_critics"] | |
| `<dwb plugin>`.prune_plan | true | |
| `<dwb plugin>`.prune_distance | 1.0 | |
| `<dwb plugin>`.debug_trajectory_details | false | |
| `<dwb plugin>`.trajectory_generator_name | "dwb_plugins::StandardTrajectoryGenerator" | |
| `<dwb plugin>`.goal_checker_name | "dwb_plugins::SimpleGoalChecker" | |
| `<dwb plugin>`.transform_tolerance | 0.1 | |
| `<dwb plugin>`.short_circuit_trajectory_evaluation | true | |
| `<dwb plugin>`.path_distance_bias | N/A | |
| `<dwb plugin>`.goal_distance_bias |  | |
| `<dwb plugin>`.occdist_scale |  | |
| `<dwb plugin>`.max_scaling_factor |  | |
| `<dwb plugin>`.scaling_speed |  | |
| `<dwb plugin>`.PathAlign.scale |  | |
| `<dwb plugin>`.GoalAlign.scale |  | |
| `<dwb plugin>`.PathDist.scale |  | |
| `<dwb plugin>`.GoalDist.scale |  | |

### publisher

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.publish_evaluation |true | |
| `<dwb plugin>`.publish_global_plan | true | |
| `<dwb plugin>`.publish_transformed_plan | true | |
| `<dwb plugin>`.publish_local_plan | true | |
| `<dwb plugin>`.publish_trajectories | true | |
| `<dwb plugin>`.publish_cost_grid_pc | false | |
| `<dwb plugin>`.marker_lifetime | 0.1 | |

### oscillation TrajectoryCritic

* `<name>`: oscillation critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.oscillation_reset_dist | 0.05 | |
| `<dwb plugin>`.`<name>`.oscillation_reset_angle | 0.2 | |
| `<dwb plugin>`.`<name>`.oscillation_reset_time | -1 | |
| `<dwb plugin>`.`<name>`.x_only_threshold | 0.05 | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### kinematic_parameters 

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.max_vel_theta | 0.0 | |
| `<dwb plugin>`.min_speed_xy | 0.0 | |
| `<dwb plugin>`.max_speed_xy | 0.0| |
| `<dwb plugin>`.min_speed_theta | 0.0 | |
| `<dwb plugin>`.min_vel_x | 0.0 | |
| `<dwb plugin>`.min_vel_y | 0.0 | |
| `<dwb plugin>`.max_vel_x | 0.0 | |
| `<dwb plugin>`.max_vel_theta | 0.0 | |
| `<dwb plugin>`.acc_lim_x | 0.0 | |
| `<dwb plugin>`.acc_lim_y | 0.0 | |
| `<dwb plugin>`.acc_lim_theta | 0.0 | |
| `<dwb plugin>`.decel_lim_x | 0.0 | |
| `<dwb plugin>`.decel_lim_y | 0.0 | |
| `<dwb plugin>`.decel_lim_theta | 0.0 | |

### xy_theta_iterator 

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.vx_samples | 20 | |
| `<dwb plugin>`.vy_samples | 5 | |
| `<dwb plugin>`.vtheta_samples | 20| |

### base_obstacle TrajectoryCritic

* `<name>`: base_obstacle critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.sum_scores | false | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### obstacle_footprint TrajectoryCritic

* `<name>`: obstacle_footprint critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.sum_scores | false | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### prefer_forward TrajectoryCritic

* `<name>`: prefer_forward critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.penalty | 1.0 | |
| `<dwb plugin>`.`<name>`.strafe_x | 0.1 | |
| `<dwb plugin>`.`<name>`.strafe_theta | 0.2 | |
| `<dwb plugin>`.`<name>`.theta_scale | 10.0 | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### twirling TrajectoryCritic

* `<name>`: twirling critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.scale | 0.0 | |

### goal_align TrajectoryCritic

* `<name>`: goal_align critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.forward_point_distance | 0.325 | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### map_grid TrajectoryCritic

* `<name>`: map_grid critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.aggregation_type | "last" | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### path_dist TrajectoryCritic

* `<name>`: path_dist critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.aggregation_type | "last" | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### path_align TrajectoryCritic

* `<name>`: path_align critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.forward_point_distance | 0.325 | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### rotate_to_goal TrajectoryCritic

* `<name>`: rotate_to_goal critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.xy_goal_tolerance | 0.25 | |
| `<dwb plugin>`.trans_stopped_velocity | 0.25 | |
| `<dwb plugin>`.slowing_factor | 5.0 | |
| `<dwb plugin>`.lookahead_time | -1 | |
| `<dwb plugin>`.`<name>`.scale | 1.0 | |

### simple_goal_checker plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.xy_goal_tolerance | 0.25 | |
| `<dwb plugin>`.yaw_goal_tolerance | 0.25 | |
| `<dwb plugin>`.stateful | true | |

### standard_traj_generator plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.sim_time | 1.7 | |
| `<dwb plugin>`.discretize_by_time | false | |
| `<dwb plugin>`.time_granularity | 0.5 | |
| `<dwb plugin>`.linear_granularity | 0.5 | |
| `<dwb plugin>`.angular_granularity | 0.025 | |
| `<dwb plugin>`.include_last_point | true | |

### limited_accel_generator plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.sim_time | N/A | |

### stopped_goal_checker plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.rot_stopped_velocity | 0.25 | |
| `<dwb plugin>`.trans_stopped_velocity | 0.25 | |

# lifecycle_manager

| Parameter | Default | Description |
| ----------| --------| ------------|
| node_names | N/A | |
| autostart | false | |

# map_saver

| Parameter | Default | Description |
| ----------| --------| ------------|
| save_map_timeout | 2000 | |
| free_thresh_default | 0.25 | |
| occupied_thresh_default | 0.65 | |

# map_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| yaml_filename | N/A | |
| topic_name | "map" | |
| frame_id | "map" | |

# planner_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| planner_plugin_ids | "GridBased" | |
| planner_plugin_types | "nav2_navfn_planner/NavfnPlanner" | |

# navfn_planner

| Parameter | Default | Description |
| ----------| --------| ------------|
| name.tolerance  | 0.5 | |
| name.use_astar | false | |
| name.allow_unknown | true | |

# waypoint_follower

| Parameter | Default | Description |
| ----------| --------| ------------|
| stop_on_failure | true | |
| loop_rate | 20 | |

# recovers

## recovery_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| costmap_topic | "local_costmap/costmap_raw" | |
| footprint_topic | "local_costmap/published_footprint" | |
| cycle_frequency | 10.0 | |
| transform_tolerance | 0.1 | |
| global_frame | "odom" | |
| robot_base_frame | "base_link" | |
| plugin_names | {"spin", "back_up", "wait"}| |
| plugin_types | {"nav2_recoveries/Spin", "nav2_recoveries/BackUp", "nav2_recoveries/Wait"} | |

## spin plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| simulate_ahead_time | 2.0 | |
| max_rotational_vel | 1.0 | |
| min_rotational_vel | 0.4 | |
| rotational_acc_lim | 3.2 | |

## back_up plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| simulate_ahead_time | 2.0 | |

# AMCL

| Parameter | Default | Description |
| ----------| --------| ------------|
| alpha1 | 0.2 | |
| alpha2 | 0.2 | |
| alpha3 | 0.2 | |
| alpha4 | 0.2 | |
| alpha5 | 0.2 | |
| base_frame_id | "base_footprint" | |
| beam_skip_distance | 0.5 | |
| beam_skip_error_threshold | 0.9 |
| beam_skip_threshold | 0.3 | |
| do_beamskip | false| |
| global_frame_id | "map" | The name of the coordinate frame published by the localization system |
| lambda_short | 0.1 | Exponential decay parameter for z_short part of model |
| laser_likelihood_max_dist | 2.0 | Maximum distance to do obstacle inflation on map, for use in likelihood_field model |
| laser_max_range | 100.0 | Maximum scan range to be considered, -1.0 will cause the laser's reported maximum range to be used |
| laser_min_range | -1 | Minimum scan range to be considered, -1.0 will cause the laser's reported minimum range to be used |
| laser_model_type | "likelihood_field" | Which model to use, either beam, likelihood_field, or likelihood_field_prob. Same as likelihood_field but incorporates the beamskip feature, if enabled |
| set_initial_pose | false | Causes AMCL to set initial pose from the initial_pose* parameters instead of waiting for the initial_pose message |
| initial_pose.x | 0.0 | X coordinate of the initial robot pose in the map frame |
| initial_pose.y | 0.0 | Y coordinate of the initial robot pose in the map frame |
| initial_pose.z | 0.0 | Z coordinate of the initial robot pose in the map frame |
| initial_pose.yaw | 0.0 | Yaw of the initial robot pose in the map frame |
| max_beams | 60 | How many evenly-spaced beams in each scan to be used when updating the filter |
| max_particles | 2000 | Maximum allowed number of particles |
| min_particles | 500 | Minimum allowed number of particles |
| odom_frame_id | "odom" | Which frame to use for odometry |
| pf_err | 0.05 | |
| pf_z | 0.99 | |
| recovery_alpha_fast | 0.0 | Exponential decay rate for the slow average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.001|
| resample_interval | 1 | Number of filter updates required before resampling |
| robot_model_type | "differential" | |
| save_pose_rate | 0.5 | Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used on subsequent runs to initialize the filter (-1.0 to disable) |
| sigma_hit | 0.2 | |
| tf_broadcast | true | Set this to false to prevent amcl from publishing the transform between the global frame and the odometry frame |
| transform_tolerance | 1.0 |  Time with which to post-date the transform that is published, to indicate that this transform is valid into the future |
| update_min_a | 0.2 | Rotational movement required before performing a filter update |
| update_min_d | 0.25 | Translational movement required before performing a filter update |
| z_hit | 0.5 | |
| z_max | 0.05 | |
| z_rand | 0.5 | |
| z_short | 0.05 | |
| always_reset_initial_pose | false | Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter (with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the last known pose to initialize |

---

# Behavior Tree Nodes

## Actions

### BT Node BackUp

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| backup_dist | -0.15 | Distance to backup |
| backup_speed | 0.025 | Speed at which to backup |
| server_name | N/A | Action server name |
| server_timeout | 10 | (milliseconds) |

### BT Node ClearEntireCostmap

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| service_name | N/A | Server name |
| server_timeout | 10 | (milliseconds) |

### BT Node ComputePathToPose

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| goal | N/A | Destination to plan to |
| planner_id | N/A | |
| server_name | N/A | Action server name |
| server_timeout | 10 | (milliseconds) |

| Output Port | Default | Description |
| ----------- | ------- | ----------- |
| path | N/A | Path created by ComputePathToPose node |

### BT Node FollowPath

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| path | N/A | Path to follow |
| controller_id | N/A | |
| server_name | N/A | Action server name |
| server_timeout | 10 | (milliseconds) |

### BT Node NavigateToPose

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| position | N/A | Position |
| orientation | N/A | Orientation |
| server_name | N/A | Action server name |
| server_timeout | 10 | (milliseconds) |

### BT Node ReinitializeGlobalLocalization

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| service_name | N/A | Server name |
| server_timeout | 10 | (milliseconds) |

### BT Node Spin

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| spin_dist | 1.57 | Spin distance |
| server_name | N/A | Action server name |
| server_timeout | 10 | (milliseconds) |

### BT Node Wait

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| wait_duration | 1 | Wait time |
| server_name | N/A | Action server name |
| server_timeout | 10 | (milliseconds) |

## Conditions

### BT Node GoalReached

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| goal | N/A | Destination |
| global_frame | "map" | Global frame |
| robot_base_frame | "base_link" | Robot base frame |

### BT Node TransformAvailable (condition)

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| child | "" | Child frame for transform |
| parent | "" | parent frame for transform |

## Controls

### BT Node RecoveryNode

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| number_of_retries | 1 | Number of retries |

## Decorators

### BT Node DistanceController

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| distance | 1.0 | Distance |
| global_frame | "map" | Global frame |
| robot_base_frame | "base_link" | Robot base frame |

### BT Node RateController

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| hz | 10.0 | Rate |