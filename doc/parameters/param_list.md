
# bt_navigator 

| Parameter | Default | Description |
| ----------| --------| ------------|
| bt_xml_filename | N/A |  |
| plugin_lib_names | ["nav2_compute_path_to_pose_action_bt_node", "nav2_follow_path_action_bt_node", "nav2_back_up_action_bt_node", "nav2_spin_action_bt_node", "nav2_wait_action_bt_node", "nav2_clear_costmap_service_bt_node", "nav2_is_stuck_condition_bt_node", "nav2_goal_reached_condition_bt_node", "nav2_initial_pose_received_condition_bt_node", "nav2_goal_updated_condition_bt_node", "nav2_reinitialize_global_localization_service_bt_node", "nav2_rate_controller_bt_node", "nav2_distance_controller_bt_node", "nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node", "nav2_round_robin_node_bt_node", "nav2_transform_available_condition_bt_node"] |  |
| transform_tolerance | 0.1 |  |
| global_frame | "map" |  |
| robot_base_frame | "base_link" |  |

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

| Parameter | Default | Description |
| ----------| --------| ------------|
| static_layer.enabled | true | |
| static_layer.subscribe_to_updates | false | |
| static_layer.subscribe_to_updates | false | |
| static_layer.map_subscribe_transient_local | true | |
| static_layer.transform_tolerance | 0.0 | |

### inflation_layer plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| inflation_layer.enabled | true | |
| inflation_layer.inflation_radius | 0.55 | |
| inflation_layer.cost_scaling_factor | 10.0 | |
| inflation_layer.inflate_unknown | false | |
| inflation_layer.inflate_around_unknown | false | |

### obstacle_layer plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| obstacle_layer.enabled | true | |
| obstacle_layer.footprint_clearing_enabled | true | |
| obstacle_layer.max_obstacle_height | 2.0 | |
| obstacle_layer.combination_method | 1 | |
| obstacle_layer.observation_sources | "" | data source topic |
| source.topic  | "" | |
| source.sensor_frame | "" | |
| source.observation_persistence | 0.0 | |
| source.expected_update_rate | 0.0 | |
| source.data_type | "LaserScan" | |
| source.min_obstacle_height | 0.0 | |
| source.max_obstacle_height | 0.0 | |
| source.inf_is_valid | false | |
| source.marking | true | |
| source.clearing | false | |
| source.obstacle_range | 2.5 | |
| source.raytrace_range | 3.0 | | 

### voxel_layer plugin

*Note*: These parameters will only get declared if `"voxel_layer"` is appended to `plugin_names` parameter and `"nav2_costmap_2d::VoxelLayer"` is appended to `plugin_types` parameter.

| Parameter | Default | Description |
| ----------| --------| ------------|
| voxel_layer.enabled | true | |
| voxel_layer.footprint_clearing_enabled | true | |
| voxel_layer.max_obstacle_height | 2.0 | |
| voxel_layer.z_voxels | 10 | |
| voxel_layer.origin_z | 0.0 | |
| voxel_layer.z_resolution | 0.2 | |
| voxel_layer.unknown_threshold | 15 | |
| voxel_layer.mark_threshold | 0 | |
| voxel_layer.combination_method | 1 | |
| voxel_layer.publish_voxel_map | false | |
| voxel_layer.observation_sources | "" | data source topic |
| source.topic  | "" | |
| source.sensor_frame | "" | |
| source.observation_persistence | 0.0 | |
| source.expected_update_rate | 0.0 | |
| source.data_type | "LaserScan" | |
| source.min_obstacle_height | 0.0 | |
| source.max_obstacle_height | 0.0 | |
| source.inf_is_valid | false | |
| source.marking | true | |
| source.clearing | false | |
| source.obstacle_range | 2.5 | |
| source.raytrace_range | 3.0 | | 

# dwb_controller

### dwb_local_planner

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.critics | | |
| dwb_plugin_name_.prune_plan | true | |
| dwb_plugin_name_.prune_distance | 1.0 | |
| dwb_plugin_name_.debug_trajectory_details | false | |
| dwb_plugin_name_.trajectory_generator_name | "dwb_plugins::StandardTrajectoryGenerator" | |
| dwb_plugin_name_.goal_checker_name | "dwb_plugins::SimpleGoalChecker" | |
| dwb_plugin_name_.transform_tolerance | 0.1 | |
| dwb_plugin_name_.short_circuit_trajectory_evaluation | true | |
| dwb_plugin_name_.path_distance_bias | N/A | |
| dwb_plugin_name_.goal_distance_bias |  | |
| dwb_plugin_name_.occdist_scale |  | |
| dwb_plugin_name_.max_scaling_factor |  | |
| dwb_plugin_name_.scaling_speed |  | |
| dwb_plugin_name_.PathAlign.scale |  | |
| dwb_plugin_name_.GoalAlign.scale |  | |
| dwb_plugin_name_.PathDist.scale |  | |
| dwb_plugin_name_.GoalDist.scale |  | |

### publisher

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.publish_evaluation |true | |
| dwb_plugin_name_.publish_global_plan | true | |
| dwb_plugin_name_.publish_transformed_plan | true | |
| dwb_plugin_name_.publish_local_plan | true | |
| dwb_plugin_name_.publish_trajectories | true | |
| dwb_plugin_name_.publish_cost_grid_pc | false | |
| dwb_plugin_name_.marker_lifetime | 0.1 | |

### oscillation

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.oscillation_reset_dist | 0.05 | |
| dwb_plugin_name_.name.oscillation_reset_angle | 0.2 | |
| dwb_plugin_name_.name.oscillation_reset_time | -1 | |
| dwb_plugin_name_.name.x_only_threshold | 0.05 | |
| dwb_plugin_name_.name.scale | 1.0 | |

### kinematic_parameters 

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.max_vel_theta | 0.0 | |
| dwb_plugin_name_.min_speed_xy | 0.0 | |
| dwb_plugin_name_.max_speed_xy | 0.0| |
| dwb_plugin_name_.min_speed_theta | 0.0 | |
| dwb_plugin_name_.min_vel_x | 0.0 | |
| dwb_plugin_name_.min_vel_y | 0.0 | |
| dwb_plugin_name_.max_vel_x | 0.0 | |
| dwb_plugin_name_.max_vel_theta | 0.0 | |
| dwb_plugin_name_.acc_lim_x | 0.0 | |
| dwb_plugin_name_.acc_lim_y | 0.0 | |
| dwb_plugin_name_.acc_lim_theta | 0.0 | |
| dwb_plugin_name_.decel_lim_x | 0.0 | |
| dwb_plugin_name_.decel_lim_y | 0.0 | |
| dwb_plugin_name_.decel_lim_theta | 0.0 | |

### xy_theta_iterator 

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.vx_samples | 20 | |
| dwb_plugin_name_.vy_samples | 5 | |
| dwb_plugin_name_.vtheta_samples | 20| |

### base_obstacle TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.sum_scores | false | |
| dwb_plugin_name_.name.scale | 1.0 | |

### obstacle_footprint TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.sum_scores | false | |
| dwb_plugin_name_.name.scale | 1.0 | |

### prefer_forward TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.penalty | 1.0 | |
| dwb_plugin_name_.name.strafe_x | 0.1 | |
| dwb_plugin_name_.name.strafe_theta | 0.2 | |
| dwb_plugin_name_.name.theta_scale | 10.0 | |
| dwb_plugin_name_.name.scale | 1.0 | |

### twirling TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.scale | 0.0 | |

### goal_align TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.forward_point_distance | 0.325 | |
| dwb_plugin_name_.name.scale | 1.0 | |

### map_grid TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.aggregation_type | "last" | |
| dwb_plugin_name_.name.scale | 1.0 | |

### path_dist TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.aggregation_type | "last" | |
| dwb_plugin_name_.name.scale | 1.0 | |

### path_align TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.name.forward_point_distance | 0.325 | |
| dwb_plugin_name_.name.scale | 1.0 | |

### rotate_to_goal TrajectoryCritic

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.xy_goal_tolerance | 0.25 | |
| dwb_plugin_name_.trans_stopped_velocity | 0.25 | |
| dwb_plugin_name_.slowing_factor | 5.0 | |
| dwb_plugin_name_.lookahead_time | -1 | |
| dwb_plugin_name_.name.scale | 1.0 | |

### simple_goal_checker plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.xy_goal_tolerance | 0.25 | |
| dwb_plugin_name_.yaw_goal_tolerance | 0.25 | |
| dwb_plugin_name_.stateful | true | |

### standard_traj_generator plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.sim_time | 1.7 | |
| dwb_plugin_name_.discretize_by_time | false | |
| dwb_plugin_name_.time_granularity | 0.5 | |
| dwb_plugin_name_.linear_granularity | 0.5 | |
| dwb_plugin_name_.angular_granularity | 0.025 | |
| dwb_plugin_name_.include_last_point | true | |

### limited_accel_generator plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.sim_time | N/A | |

### stopped_goal_checker plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| dwb_plugin_name_.rot_stopped_velocity | 0.25 | |
| dwb_plugin_name_.trans_stopped_velocity | 0.25 | |

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
| always_reset_initial_pose | false | Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter (with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the last known pose to initialize | --!>