.. _docs_tunable-params:

Tunable Parameters
##################

Below is a list of all tunable parameters per node with some example values.
Only relevant nodes are shown.

Also, the `nav2_bringup pkg`_ contains a `nav2_params file`_ with the default values to use for TB3.

.. rst-class:: content-collapse

AMCL
====

.. code-block:: yaml

    amcl:
      ros__parameters:
        alpha1: 0.2
        alpha2: 0.2
        alpha3: 0.2
        alpha4: 0.2
        alpha5: 0.2
        always_reset_initial_pose: false
        base_frame_id: base_footprint
        beam_skip_distance: 0.5
        beam_skip_error_threshold: 0.9
        beam_skip_threshold: 0.3
        do_beamskip: false
        global_frame_id: map
        lambda_short: 0.1
        laser_likelihood_max_dist: 2.0
        laser_max_range: 100.0
        laser_min_range: -1.0
        laser_model_type: likelihood_field
        max_beams: 60
        max_particles: 2000
        min_particles: 500
        odom_frame_id: odom
        pf_err: 0.05
        pf_z: 0.99
        recovery_alpha_fast: 0.0
        recovery_alpha_slow: 0.0
        resample_interval: 1
        robot_model_type: differential
        save_pose_rate: 0.5
        set_initial_pose: false
        sigma_hit: 0.2
        tf_broadcast: true
        transform_tolerance: 1.0
        update_min_a: 0.2
        update_min_d: 0.25
        use_sim_time: true
        z_hit: 0.5
        z_max: 0.05
        z_rand: 0.5
        z_short: 0.05


.. rst-class:: content-collapse

Controller Server
=================

Hosts the `DWB` controller.

.. code-block:: yaml

    controller_server:
      ros__parameters:
        BaseObstacle:
          scale: 0.02
          sum_scores: false
        BaseObstacle/class: BaseObstacle
        GoalAlign:
          aggregation_type: last
          scale: 0.0
        GoalAlign/class: GoalAlign
        GoalDist:
          aggregation_type: last
          scale: 24.0
        GoalDist/class: GoalDist
        Oscillation:
          scale: 1.0
          x_only_threshold: 0.05
        Oscillation/class: Oscillation
        PathAlign:
          aggregation_type: last
          scale: 0.0
        PathAlign/class: PathAlign
        PathDist:
          aggregation_type: last
          scale: 32.0
        PathDist/class: PathDist
        RotateToGoal:
          scale: 32.0
        RotateToGoal/class: RotateToGoal
        acc_lim_theta: 3.2
        acc_lim_x: 2.5
        acc_lim_y: 0.0
        controller_frequency: 20.0
        critics:
        - RotateToGoal
        - Oscillation
        - BaseObstacle
        - GoalAlign
        - PathAlign
        - PathDist
        - GoalDist
        debug_trajectory_details: true
        decel_lim_theta: -3.2
        decel_lim_x: -2.5
        decel_lim_y: 0.0
        discretize_by_time: false
        goal_checker_name: dwb_plugins::SimpleGoalChecker
        local_controller_plugin: dwb_core::DWBLocalPlanner
        max_speed_xy: 0.26
        max_vel_theta: 1.0
        max_vel_x: 0.26
        max_vel_y: 0.0
        min_speed_theta: 0.0
        min_speed_xy: 0.0
        min_theta_velocity_threshold: 0.001
        min_vel_x: 0.0
        min_vel_y: 0.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        prune_distance: 1.0
        prune_plan: true
        publish_cost_grid_pc: false
        publish_evaluation: true
        publish_global_plan: true
        publish_local_plan: true
        publish_trajectories: true
        publish_transformed_plan: true
        sim_time: 1.7
        trajectory_generator_name: dwb_plugins::StandardTrajectoryGenerator
        transform_tolerance: 0.2
        use_sim_time: true
        vx_samples: 20
        vy_samples: 5
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 0.25


.. rst-class:: content-collapse

Local Costmap
=================

Hosted on the `PlannerServer` node.

.. code-block:: yaml

    local_costmap:
      ros__parameters:
        always_send_full_costmap: true
        clearable_layers:
        - obstacle_layer
        footprint: '[]'
        footprint_padding: 0.01
        global_frame: odom
        height: 3
        inflation_layer:
          cost_scaling_factor: 3.0
          enabled: true
          inflate_unknown: false
          inflation_radius: 0.55
        lethal_cost_threshold: 100
        map_topic: /map
        observation_sources: ''
        obstacle_layer:
          combination_method: 1
          enabled: true
          footprint_clearing_enabled: true
          max_obstacle_height: 2.0
          observation_sources: scan
          scan:
            clearing: true
            data_type: LaserScan
            expected_update_rate: 0.0
            inf_is_valid: false
            marking: true
            max_obstacle_height: 2.0
            min_obstacle_height: 0.0
            observation_persistence: 0.0
            obstacle_range: 2.5
            raytrace_range: 3.0
            sensor_frame: ''
            topic: /scan
        origin_x: 0.0
        origin_y: 0.0
        plugin_names:
        - obstacle_layer
        - voxel_layer
        - inflation_layer
        plugin_types:
        - nav2_costmap_2d::ObstacleLayer
        - nav2_costmap_2d::VoxelLayer
        - nav2_costmap_2d::InflationLayer
        publish_frequency: 1.0
        resolution: 0.05
        robot_base_frame: base_link
        robot_radius: 0.22
        rolling_window: true
        track_unknown_space: false
        transform_tolerance: 0.3
        trinary_costmap: true
        unknown_cost_value: 255
        update_frequency: 5.0
        use_maximum: false
        use_sim_time: true
        voxel_layer:
          combination_method: 1
          enabled: true
          footprint_clearing_enabled: true
          mark_threshold: 0
          max_obstacle_height: 2.0
          observation_sources: pointcloud
          origin_z: 0.0
          pointcloud:
            clearing: true
            data_type: PointCloud2
            expected_update_rate: 0.0
            inf_is_valid: false
            marking: true
            max_obstacle_height: 2.0
            min_obstacle_height: 0.0
            observation_persistence: 0.0
            obstacle_range: 2.5
            raytrace_range: 3.0
            sensor_frame: ''
            topic: /intel_realsense_r200_depth/points
          publish_voxel_map: true
          unknown_threshold: 15
          z_resolution: 0.2
          z_voxels: 10
        width: 3


.. rst-class:: content-collapse

Planner Server
=================

Hosts the `NAVFN` controller.

.. code-block:: yaml

    planner_server:
      ros__parameters:
        allow_unknown: true
        planner_plugin: nav2_navfn_planner/NavfnPlanner
        tolerance: 0.0
        use_astar: false
        use_sim_time: true


.. rst-class:: content-collapse

Global Costmap
=================

Hosted on the `ControllerServer` node.

.. code-block:: yaml

    global_costmap:
      ros__parameters:
        always_send_full_costmap: true
        clearable_layers:
        - obstacle_layer
        footprint: '[]'
        footprint_padding: 0.01
        global_frame: map
        height: 10
        inflation_layer:
          cost_scaling_factor: 10.0
          enabled: true
          inflate_unknown: false
          inflation_radius: 0.55
        lethal_cost_threshold: 100
        map_topic: /map
        observation_sources: ''
        obstacle_layer:
          combination_method: 1
          enabled: true
          footprint_clearing_enabled: true
          max_obstacle_height: 2.0
          observation_sources: scan
          scan:
            clearing: true
            data_type: LaserScan
            expected_update_rate: 0.0
            inf_is_valid: false
            marking: true
            max_obstacle_height: 2.0
            min_obstacle_height: 0.0
            observation_persistence: 0.0
            obstacle_range: 2.5
            raytrace_range: 3.0
            sensor_frame: ''
            topic: /scan
        origin_x: 0.0
        origin_y: 0.0
        plugin_names:
        - static_layer
        - obstacle_layer
        - voxel_layer
        - inflation_layer
        plugin_types:
        - nav2_costmap_2d::StaticLayer
        - nav2_costmap_2d::ObstacleLayer
        - nav2_costmap_2d::VoxelLayer
        - nav2_costmap_2d::InflationLayer
        publish_frequency: 1.0
        resolution: 0.1
        robot_base_frame: base_link
        robot_radius: 0.22
        rolling_window: false
        static_layer:
          enabled: true
          map_subscribe_transient_local: true
          subscribe_to_updates: false
        track_unknown_space: false
        transform_tolerance: 0.3
        trinary_costmap: true
        unknown_cost_value: 255
        update_frequency: 5.0
        use_maximum: false
        use_sim_time: true
        voxel_layer:
          combination_method: 1
          enabled: true
          footprint_clearing_enabled: true
          mark_threshold: 0
          max_obstacle_height: 2.0
          observation_sources: pointcloud
          origin_z: 0.0
          pointcloud:
            clearing: true
            data_type: PointCloud2
            expected_update_rate: 0.0
            inf_is_valid: false
            marking: true
            max_obstacle_height: 2.0
            min_obstacle_height: 0.0
            observation_persistence: 0.0
            obstacle_range: 2.5
            raytrace_range: 3.0
            sensor_frame: ''
            topic: /intel_realsense_r200_depth/points
          publish_voxel_map: true
          unknown_threshold: 15
          z_resolution: 0.2
          z_voxels: 10
        width: 10


.. rst-class:: content-collapse

Map Server
=================

.. code-block:: yaml

    map_server:
      ros__parameters:
        use_sim_time: true
        yaml_filename: turtlebot3_world.yaml


.. rst-class:: content-collapse

Recovery Server
=================

Hosts multiple recovery actions

.. code-block:: yaml

    recoveries_server:
      ros__parameters:
        costmap_topic: local_costmap/costmap_raw
        cycle_frequency: 10.0
        footprint_topic: local_costmap/published_footprint
        max_rotational_vel: 1.0
        min_rotational_vel: 0.4
        plugin_names:
        - Spin
        - BackUp
        - Wait
        plugin_types:
        - nav2_recoveries/Spin
        - nav2_recoveries/BackUp
        - nav2_recoveries/Wait
        rotational_acc_lim: 3.2
        simulate_ahead_time: 2.0
        use_sim_time: true


.. rst-class:: content-collapse

BT Navigator
=================
Behavior-Tree-based Navigator

.. code-block:: yaml

    bt_navigator:
      ros__parameters:
        bt_xml_filename: navigate_w_replanning_and_recovery.xml
        use_sim_time: true
