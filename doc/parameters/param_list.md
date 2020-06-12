*NOTE: <> means plugin are namespaced by a name/id parameterization. The bracketed names may change due to your application configuration*

# bt_navigator

| Parameter | Default | Description |
| ----------| --------| ------------|
| default_bt_xml_filename | N/A | path to the default behavior tree XML description |
| plugin_lib_names | ["nav2_compute_path_to_pose_action_bt_node", "nav2_follow_path_action_bt_node", "nav2_back_up_action_bt_node", "nav2_spin_action_bt_node", "nav2_wait_action_bt_node", "nav2_clear_costmap_service_bt_node", "nav2_is_stuck_condition_bt_node", "nav2_goal_reached_condition_bt_node", "nav2_initial_pose_received_condition_bt_node", "nav2_goal_updated_condition_bt_node", "nav2_reinitialize_global_localization_service_bt_node", "nav2_rate_controller_bt_node", "nav2_distance_controller_bt_node", "nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node", "nav2_round_robin_node_bt_node", "nav2_transform_available_condition_bt_node"] | list of behavior tree node shared libraries |
| transform_tolerance | 0.1 | TF transform tolerance |
| global_frame | "map" | Reference frame |
| robot_base_frame | "base_link" | Robot base frame |
| odom_topic | "odom" | Odometry topic |

# costmaps

## Node: costmap_2d_ros

Namespace: /parent_ns/local_ns

| Parameter | Default | Description |
| ----------| --------| ------------|
| always_send_full_costmap | false | Whether to send full costmap every update, rather than updates |
| footprint_padding | 0.01 | Amount to pad footprint (m) |
| footprint | "[]" | Ordered set of footprint points, must be closed set |
| global_frame | "map" | Reference frame |
| height | 5 | Height of costmap (m) |
| width | 5 | Width of costmap (m) |
| lethal_cost_threshold | 100 | Minimum cost of an occupancy grid map to be considered a lethal obstacle |
| map_topic | "parent_namespace/map" | Topic of map from map_server or SLAM |
| observation_sources | [""] | List of sources of sensors, to be used if not specified in plugin specific configurations |
| origin_x | 0.0 | X origin of the costmap relative to width (m) |
| origin_y | 0.0 | Y origin of the costmap relative to height (m) |
| plugin_names | {"static_layer", "obstacle_layer", "inflation_layer"} | List of mapped plugin names for parameter namespaces and names |
| plugin_types | {"nav2_costmap_2d::StaticLayer", "nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::InflationLayer"} | List of registered plugins to map to names and load |
| publish_frequency | 1.0 | Frequency to publish costmap to topic |
| resolution | 0.1 | Resolution of 1 pixel of the costmap, in meters |
| robot_base_frame | "base_link" | Robot base frame |
| robot_radius| 0.1 | Robot radius to use, if footprint coordinates not provided |
| rolling_window | false | Whether costmap should roll with robot base frame |
| track_unknown_space | false | If false, treats unknown space as free space, else as unknown space |
| transform_tolerance | 0.3 | TF transform tolerance |
| trinary_costmap | true | If occupancy grid map should be interpreted as only 3 values (free, occupied, unknown) or with its stored values |
| unknown_cost_value | 255 | Cost of unknown space if tracking it |
| update_frequency | 5.0 | Costmap update frequency |
| use_maximum | false | whether when combining costmaps to use the maximum cost or override |
| clearable_layers | ["obstacle_layer"] | Layers that may be cleared using the clearing service |

## static_layer plugin

* `<static layer>`: Name corresponding to the `nav2_costmap_2d::StaticLayer` plugin. This name gets defined in `plugin_names`, default value is `static_layer`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<static layer>`.enabled | true | Whether it is enabled |
| `<static layer>`.subscribe_to_updates | false | Subscribe to static map updates after receiving first |
| `<static layer>`.map_subscribe_transient_local | true | QoS settings for map topic |
| `<static layer>`.transform_tolerance | 0.0 | TF tolerance |

## inflation_layer plugin

* `<inflation layer>`: Name corresponding to the `nav2_costmap_2d::InflationLayer` plugin. This name gets defined in `plugin_names`, default value is `inflation_layer`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<inflation layer>`.enabled | true | Whether it is enabled |
| `<inflation layer>`.inflation_radius | 0.55 | Radius to inflate costmap around lethal obstacles |
| `<inflation layer>`.cost_scaling_factor | 10.0 | Exponential decay factor across inflation radius |
| `<inflation layer>`.inflate_unknown | false | Whether to inflate unknown cells as if lethal |
| `<inflation layer>`.inflate_around_unknown | false | Whether to inflate unknown cells  |

## obstacle_layer plugin

* `<obstacle layer>`: Name corresponding to the `nav2_costmap_2d::ObstacleLayer` plugin. This name gets defined in `plugin_names`, default value is `obstacle_layer`
* `<data source>`: Name of a source provided in ``<obstacle layer>`.observation_sources`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<obstacle layer>`.enabled | true | Whether it is enabled |
| `<obstacle layer>`.footprint_clearing_enabled | true | Clear any occupied cells under robot footprint |
| `<obstacle layer>`.max_obstacle_height | 2.0 | Maximum height to add return to occupancy grid |
| `<obstacle layer>`.combination_method | 1 | Enum for method to add data to master costmap, default to maximum |
| `<obstacle layer>`.observation_sources | "" | namespace of sources of data |
| `<data source>`.topic  | "" | Topic of data |
| `<data source>`.sensor_frame | "" | frame of sensor, to use if not provided by message |
| `<data source>`.observation_persistence | 0.0 | How long to store messages in a buffer to add to costmap before removing them (s) |
| `<data source>`.expected_update_rate | 0.0 | Expected rate to get new data from sensor |
| `<data source>`.data_type | "LaserScan" | Data type of input, LaserScan or PointCloud2 |
| `<data source>`.min_obstacle_height | 0.0 | Minimum height to add return to occupancy grid |
| `<data source>`.max_obstacle_height | 0.0 | Maximum height to add return to occupancy grid for this source |
| `<data source>`.inf_is_valid | false | Are infinite returns from laser scanners valid measurements |
| `<data source>`.marking | true | Whether source should mark in costmap |
| `<data source>`.clearing | false | Whether source should raytrace clear in costmap |
| `<data source>`.obstacle_range | 2.5 | Maximum range to mark obstacles in costmap |
| `<data source>`.raytrace_range | 3.0 | Maximum range to raytrace clear obstacles from costmap | 

## voxel_layer plugin

* `<voxel layer>`: Name corresponding to the `nav2_costmap_2d::VoxelLayer` plugin. This name gets defined in `plugin_names`
* `<data source>`: Name of a source provided in `<voxel layer>`.observation_sources`

*Note*: These parameters will only get declared if a `<voxel layer>` name such as `voxel_layer` is appended to `plugin_names` parameter and `"nav2_costmap_2d::VoxelLayer"` is appended to `plugin_types` parameter.

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<voxel layer>`.enabled | true | Whether it is enabled |
| `<voxel layer>`.footprint_clearing_enabled | true | Clear any occupied cells under robot footprint |
| `<voxel layer>`.max_obstacle_height | 2.0 | Maximum height to add return to occupancy grid |
| `<voxel layer>`.z_voxels | 10 | Number of voxels high to mark, maximum 16|
| `<voxel layer>`.origin_z | 0.0 | Where to start marking voxels (m) |
| `<voxel layer>`.z_resolution | 0.2 | Resolution of voxels in height (m) |
| `<voxel layer>`.unknown_threshold | 15 | Minimum number of empty voxels in a column to mark as unknown in 2D occupancy grid |
| `<voxel layer>`.mark_threshold | 0 | Minimum number of voxels in a column to mark as occupied in 2D occupancy grid |
| `<voxel layer>`.combination_method | 1 | Enum for method to add data to master costmap, default to maximum |
| `<voxel layer>`.publish_voxel_map | false | Whether to publish 3D voxel grid, computationally expensive |
| `<voxel layer>`.observation_sources | "" | namespace of sources of data |
| `<data source>`.topic  | "" | Topic of data |
| `<data source>`.sensor_frame | "" | frame of sensor, to use if not provided by message |
| `<data source>`.observation_persistence | 0.0 | How long to store messages in a buffer to add to costmap before removing them (s) |
| `<data source>`.expected_update_rate | 0.0 | Expected rate to get new data from sensor |
| `<data source>`.data_type | "LaserScan" | Data type of input, LaserScan or PointCloud2 |
| `<data source>`.min_obstacle_height | 0.0 | Minimum height to add return to occupancy grid |
| `<data source>`.max_obstacle_height | 0.0 | Maximum height to add return to occupancy grid for this source |
| `<data source>`.inf_is_valid | false | Are infinite returns from laser scanners valid measurements |
| `<data source>`.marking | true | Whether source should mark in costmap |
| `<data source>`.clearing | false | Whether source should raytrace clear in costmap |
| `<data source>`.obstacle_range | 2.5 | Maximum range to mark obstacles in costmap |
| `<data source>`.raytrace_range | 3.0 | Maximum range to raytrace clear obstacles from costmap | 

# controller_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| controller_frequency | 20.0 | Frequency to run controller |
| controller_plugin_ids | ["FollowPath"] | List of mapped names for controller plugins for processing requests and parameters |
| controller_plugin_types | ["dwb_core::DWBLocalPlanner"] | List of registered plugins to load |
| min_x_velocity_threshold | 0.0001 | Minimum X velocity to use (m/s) |
| min_y_velocity_threshold | 0.0001 | Minimum Y velocity to use (m/s) |
| min_theta_velocity_threshold | 0.0001 | Minimum angular velocity to use (rad/s) |
| required_movement_radius | 0.5 | Minimum amount a robot must move to be progressing to goal (m) |
| movement_time_allowance | 10.0 | Maximum amount of time a robot has to move the minimum radius (s) |

# dwb_controller

* `<dwb plugin>`: DWB plugin name defined in `controller_plugin_ids` in the controller_server parameters

## dwb_local_planner

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.critics | N/A | List of critic plugins to use |
| `<dwb plugin>`.default_critic_namespaces | ["dwb_critics"] | Namespaces to load critics in |
| `<dwb plugin>`.prune_plan | true | Whether to prune the path of old, passed points |
| `<dwb plugin>`.prune_distance | 1.0 | Distance (m) to prune backward until |
| `<dwb plugin>`.debug_trajectory_details | false | Publish debug information |
| `<dwb plugin>`.trajectory_generator_name | "dwb_plugins::StandardTrajectoryGenerator" | Trajectory generator plugin name |
| `<dwb plugin>`.goal_checker_name | "dwb_plugins::SimpleGoalChecker" | Goal checker plugin name |
| `<dwb plugin>`.transform_tolerance | 0.1 | TF transform tolerance |
| `<dwb plugin>`.short_circuit_trajectory_evaluation | true | Stop evaluating scores after best score is found |
| `<dwb plugin>`.path_distance_bias | N/A | Old version of `PathAlign.scale`, use that instead |
| `<dwb plugin>`.goal_distance_bias | N/A | Old version of `GoalAlign.scale`, use that instead |
| `<dwb plugin>`.occdist_scale | N/A | Old version of `ObstacleFootprint.scale`, use that instead |
| `<dwb plugin>`.max_scaling_factor | N/A | Old version of `ObstacleFootprint.max_scaling_factor`, use that instead |
| `<dwb plugin>`.scaling_speed | N/A | Old version of `ObstacleFootprint.scaling_speed`, use that instead |
| `<dwb plugin>`.PathAlign.scale | 32.0 | Scale for path align critic, overriding local default |
| `<dwb plugin>`.GoalAlign.scale | 24.0 | Scale for goal align critic, overriding local default |
| `<dwb plugin>`.PathDist.scale | 32.0 | Scale for path distance critic, overriding local default |
| `<dwb plugin>`.GoalDist.scale | 24.0 | Scale for goal distance critic, overriding local default |

## publisher

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.publish_evaluation |true | Whether to publish the local plan evaluation |
| `<dwb plugin>`.publish_global_plan | true | Whether to publish the global plan |
| `<dwb plugin>`.publish_transformed_plan | true | Whether to publish the global plan in the odometry frame |
| `<dwb plugin>`.publish_local_plan | true | Whether to publish the local planner's plan |
| `<dwb plugin>`.publish_trajectories | true | Whether to publish debug trajectories |
| `<dwb plugin>`.publish_cost_grid_pc | false | Whether to publish the cost grid |
| `<dwb plugin>`.marker_lifetime | 0.1 | How long for the marker to remain |

## oscillation TrajectoryCritic

* `<name>`: oscillation critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.oscillation_reset_dist | 0.05 | Minimum distance to move to reset oscillation watchdog (m) |
| `<dwb plugin>`.`<name>`.oscillation_reset_angle | 0.2 | Minimum angular distance to move to reset watchdog (rad) |
| `<dwb plugin>`.`<name>`.oscillation_reset_time | -1 | Duration when a reset may be called. If -1, cannot be reset. |
| `<dwb plugin>`.`<name>`.x_only_threshold | 0.05 | Threshold to check in the X velocity direction |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weighed scale for critic |

## kinematic_parameters 

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.max_vel_theta | 0.0 | Maximum angular velocity (rad/s) |
| `<dwb plugin>`.min_speed_xy | 0.0 | Minimum translational speed (m/s) |
| `<dwb plugin>`.max_speed_xy | 0.0 | Maximum translational speed (m/s) |
| `<dwb plugin>`.min_speed_theta | 0.0 | Minimum angular speed (rad/s) |
| `<dwb plugin>`.min_vel_x | 0.0 | Minimum velocity X (m/s) |
| `<dwb plugin>`.min_vel_y | 0.0 | Minimum velocity Y (m/s) |
| `<dwb plugin>`.max_vel_x | 0.0 | Maximum velocity X (m/s) |
| `<dwb plugin>`.max_vel_y | 0.0 | Maximum velocity Y (m/s) |
| `<dwb plugin>`.acc_lim_x | 0.0 | Maximum acceleration X (m/s^2) |
| `<dwb plugin>`.acc_lim_y | 0.0 | Maximum acceleration Y (m/s^2) |
| `<dwb plugin>`.acc_lim_theta | 0.0 | Maximum acceleration rotation (rad/s^2) |
| `<dwb plugin>`.decel_lim_x | 0.0 | Maximum deceleration X (m/s^2) |
| `<dwb plugin>`.decel_lim_y | 0.0 | Maximum deceleration Y (m/s^2) |
| `<dwb plugin>`.decel_lim_theta | 0.0 | Maximum deceleration rotation (rad/s^2) |

## xy_theta_iterator 

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.vx_samples | 20 | Number of  velocity samples in the X velocity direction |
| `<dwb plugin>`.vy_samples | 5 | Number of velocity samples in the Y velocity direction |
| `<dwb plugin>`.vtheta_samples | 20 | Number of velocity samples in the angular directions |

## base_obstacle TrajectoryCritic

* `<name>`: base_obstacle critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.sum_scores | false | Whether to allow for scores to be sumed up |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## obstacle_footprint TrajectoryCritic

* `<name>`: obstacle_footprint critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.sum_scores | false | Whether to allow for scores to be sumed up |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## prefer_forward TrajectoryCritic

* `<name>`: prefer_forward critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.penalty | 1.0 | Penalty to apply to backward motion |
| `<dwb plugin>`.`<name>`.strafe_x | 0.1 | Minimum X velocity before penalty |
| `<dwb plugin>`.`<name>`.strafe_theta | 0.2 | Minimum angular velocity before penalty |
| `<dwb plugin>`.`<name>`.theta_scale | 10.0 | Weight for angular velocity component |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## twirling TrajectoryCritic

* `<name>`: twirling critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.scale | 0.0 | Weight scale |

## goal_dist TrajectoryCritic

| `<dwb plugin>`.`<name>`.aggregation_type | "last" | last, sum, or product combination methods |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## goal_align TrajectoryCritic

* `<name>`: goal_align critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.forward_point_distance | 0.325 | Point in front of robot to look ahead to compute angular change from |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |
| `<dwb plugin>`.`<name>`.aggregation_type | "last" | last, sum, or product combination methods |

## map_grid TrajectoryCritic

* `<name>`: map_grid critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.aggregation_type | "last" | last, sum, or product combination methods |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## path_dist TrajectoryCritic

* `<name>`: path_dist critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.aggregation_type | "last" | last, sum, or product combination methods |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## path_align TrajectoryCritic

* `<name>`: path_align critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.`<name>`.forward_point_distance | 0.325 | Point in front of robot to look ahead to compute angular change from |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |
| `<dwb plugin>`.`<name>`.aggregation_type | "last" | last, sum, or product combination methods |

## rotate_to_goal TrajectoryCritic

* `<name>`: rotate_to_goal critic name defined in `<dwb plugin>.critics`

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.xy_goal_tolerance | 0.25 | Tolerance to meet goal completion criteria (m) |
| `<dwb plugin>`.trans_stopped_velocity | 0.25 | Velocity below is considered to be stopped at tolerance met (rad/s) |
| `<dwb plugin>`.slowing_factor | 5.0 | Factor to slow robot motion by while rotating to goal |
| `<dwb plugin>`.lookahead_time | -1 | If > 0, amount of time to look forward for a collision for. |
| `<dwb plugin>`.`<name>`.scale | 1.0 | Weight scale |

## simple_goal_checker plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.xy_goal_tolerance | 0.25 | Tolerance to meet goal completion criteria (m) |
| `<dwb plugin>`.yaw_goal_tolerance | 0.25 | Tolerance to meet goal completion criteria (rad) |
| `<dwb plugin>`.stateful | true | Whether to check for XY position tolerance after rotating to goal orientation in case of minor localization changes |

## standard_traj_generator plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.sim_time | 1.7 | Time to simulate ahead by (s) |
| `<dwb plugin>`.discretize_by_time | false | If true, forward simulate by time. If False, forward simulate by linear and angular granularity |
| `<dwb plugin>`.time_granularity | 0.5 | Time ahead to project |
| `<dwb plugin>`.linear_granularity | 0.5 | Linear distance forward to project |
| `<dwb plugin>`.angular_granularity | 0.025 | Angular distance to project |
| `<dwb plugin>`.include_last_point | true | Whether to include the last pose in the trajectory |

## limited_accel_generator plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.sim_time | N/A | Time to simulate ahead by (s) |

## stopped_goal_checker plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<dwb plugin>`.rot_stopped_velocity | 0.25 | Velocity below is considered to be stopped at tolerance met (rad/s) |
| `<dwb plugin>`.trans_stopped_velocity | 0.25 | Velocity below is considered to be stopped at tolerance met (m/s) |

# lifecycle_manager

| Parameter | Default | Description |
| ----------| --------| ------------|
| node_names | N/A | Ordered list of node names to bringup through lifecycle transition |
| autostart | false | Whether to transition nodes to active state on startup |

# map_server

## map_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| save_map_timeout | 2000 | Timeout to attempt to save map with (ms) |
| free_thresh_default | 0.25 | Free space maximum threshold for occupancy grid |
| occupied_thresh_default | 0.65 | Occupied space minimum threshhold for occupancy grid |

## map_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| yaml_filename | N/A | Path to map yaml file |
| topic_name | "map" | topic  to publish loaded map to |
| frame_id | "map" | Frame to publish loaded map in |

# planner_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| planner_plugin_ids | ["GridBased"] | List of Mapped plugin names for parameters and processing requests |
| planner_plugin_types | ["nav2_navfn_planner/NavfnPlanner"] | List of registered pluginlib planner types to load |

# navfn_planner

* `<name>`: Corresponding planner plugin ID for this type

| Parameter | Default | Description |
| ----------| --------| ------------|
| `<name>`.tolerance  | 0.5 | Tolerance in meters between requested goal pose and end of path |
| `<name>`.use_astar | false | Whether to use A*, if false, uses Dijstra's expansion |
| `<name>`.allow_unknown | true | Whether to allow planning in unknown space |

# waypoint_follower

| Parameter | Default | Description |
| ----------| --------| ------------|
| stop_on_failure | true | Whether to fail action task if a single waypoint fails. If false, will continue to next waypoint. |
| loop_rate | 20 | Rate to check for results from current navigation task |

# recoveries

## recovery_server

| Parameter | Default | Description |
| ----------| --------| ------------|
| costmap_topic | "local_costmap/costmap_raw" | Raw costmap topic for collision checking |
| footprint_topic | "local_costmap/published_footprint" | Topic for footprint in the costmap frame |
| cycle_frequency | 10.0 | Frequency to run recovery plugins |
| transform_tolerance | 0.1 | TF transform tolerance |
| global_frame | "odom" | Reference frame |
| robot_base_frame | "base_link" | Robot base frame |
| plugin_names | {"spin", "back_up", "wait"}| List of plugin names to use, also matches action server names |
| plugin_types | {"nav2_recoveries/Spin", "nav2_recoveries/BackUp", "nav2_recoveries/Wait"} | List of registered plugin to map to names |

## spin plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| simulate_ahead_time | 2.0 | Time to look ahead for collisions (s) |
| max_rotational_vel | 1.0 | Maximum rotational velocity (rad/s) |
| min_rotational_vel | 0.4 | Minimum rotational velocity (rad/s) |
| rotational_acc_lim | 3.2 | maximum rotational acceleration (rad/s^2) |

## back_up plugin

| Parameter | Default | Description |
| ----------| --------| ------------|
| simulate_ahead_time | 2.0 | Time to look ahead for collisions (s) |

# AMCL

| Parameter | Default | Description |
| ----------| --------| ------------|
| alpha1 | 0.2 | Expected process noise in odometry's rotation estimate from rotation |
| alpha2 | 0.2 | Expected process noise in odometry's rotation estimate from translation |
| alpha3 | 0.2 | Expected process noise in odometry's translation estimate from translation |
| alpha4 | 0.2 | Expected process noise in odometry's translation estimate from rotation |
| alpha5 | 0.2 | For Omni models only: translation noise |
| base_frame_id | "base_footprint" | Base frame |
| beam_skip_distance | 0.5 | Ignore beams that most particles disagree with in Likelihood field model. Maximum distance to consider skipping for (m) |
| beam_skip_error_threshold | 0.9 | Percentage of beams after not matching map to force full update due to bad convergance |
| beam_skip_threshold | 0.3 | Percentage of beams required to skip |
| do_beamskip | false | Whether to do beam skipping in Likelihood field model. |
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
| pf_err | 0.05 | Particle Filter population error |
| pf_z | 0.99 | Particle filter population density |
| recovery_alpha_fast | 0.0 | Exponential decay rate for the slow average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.001|
| resample_interval | 1 | Number of filter updates required before resampling |
| robot_model_type | "differential" | |
| save_pose_rate | 0.5 | Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used on subsequent runs to initialize the filter (-1.0 to disable) |
| sigma_hit | 0.2 | Standard deviation for Gaussian model used in z_hit part of the model. |
| tf_broadcast | true | Set this to false to prevent amcl from publishing the transform between the global frame and the odometry frame |
| transform_tolerance | 1.0 |  Time with which to post-date the transform that is published, to indicate that this transform is valid into the future |
| update_min_a | 0.2 | Rotational movement required before performing a filter update |
| update_min_d | 0.25 | Translational movement required before performing a filter update |
| z_hit | 0.5 | Mixture weight for z_hit part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand. |
| z_max | 0.05 | Mixture weight for z_max part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand. |
| z_rand | 0.5 | Mixture weight for z_rand part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand. |
| z_short | 0.05 | Mixture weight for z_short part of model, sum of all used z weight must be 1. Beam uses all 4, likelihood model uses z_hit and z_rand. |
| always_reset_initial_pose | false | Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter (with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the last known pose to initialize |

---

# Behavior Tree Nodes

## Actions

### BT Node BackUp

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| backup_dist | -0.15 | Total distance to backup |
| backup_speed | 0.025 | Backup speed |
| server_name | N/A | Action server name |
| server_timeout | 10 | Action server timeout (ms) |

### BT Node ClearEntireCostmap

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| service_name | N/A | Server name |
| server_timeout | 10 | Action server timeout (ms) |

### BT Node ComputePathToPose

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| goal | N/A | Goal pose |
| planner_id | N/A | Mapped name to the planner plugin type to use, e.g. GridBased |
| server_name | N/A | Action server name |
| server_timeout | 10 | Action server timeout (ms) |

| Output Port | Default | Description |
| ----------- | ------- | ----------- |
| path | N/A | Path created by action server |

### BT Node FollowPath

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| path | N/A | Path to follow |
| controller_id | N/A | Mapped name of the controller plugin type to use, e.g. FollowPath |
| server_name | N/A | Action server name |
| server_timeout | 10 | Action server timeout (ms) |

### BT Node NavigateToPose

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| position | N/A | Position |
| orientation | N/A | Orientation |
| server_name | N/A | Action server name |
| server_timeout | 10 | Action server timeout (ms) |

### BT Node ReinitializeGlobalLocalization

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| service_name | N/A | Server name |
| server_timeout | 10 | Action server timeout (ms) |

### BT Node Spin

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| spin_dist | 1.57 | Spin distance (radians) |
| server_name | N/A | Action server name |
| server_timeout | 10 | Action server timeout (ms) |

### BT Node Wait

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| wait_duration | 1 | Wait time (s) |
| server_name | N/A | Action server name |
| server_timeout | 10 | Action server timeout (ms) |

## Conditions

### BT Node GoalReached

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| goal | N/A | Destination to check |
| global_frame | "map" | Reference frame |
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
| distance | 1.0 | Distance (m) |
| global_frame | "map" | Reference frame |
| robot_base_frame | "base_link" | Robot base frame |

### BT Node RateController

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| hz | 10.0 | Rate to throttle |

### BT Node SpeedController

| Input Port | Default | Description |
| ---------- | ------- | ----------- |
| min_rate | 0.1 | Minimum rate (hz) |
| max_rate | 1.0 | Maximum rate (hz) |
| min_speed | 0.0 | Minimum speed (m/s) |
| max_speed | 0.5 | Maximum speed (m/s) |
| filter_duration | 0.3 | Duration (secs) for velocity smoothing filter |
