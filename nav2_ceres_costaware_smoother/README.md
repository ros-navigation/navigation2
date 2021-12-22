# Ceres Cost-Aware Smoother

A smoother plugin for `nav2_smoother` based on the original deprecated smoother in `nav2_smac_planner` and put into operational state by (**RoboTech Vision**)[https://robotechvision.com/]. Suitable for applications which need planned global path to be pushed away from obstacles and/or for Reeds-Shepp motion models. Example of configuration (see indoor_navigation package of this repo for a full launch configuration):

```
smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["SmoothPath"]
    optimization_length: 10.0
    optimization_length_backwards: 5.0

    SmoothPath:
      plugin: "nav2_ceres_costaware_smoother/CeresCostawareSmoother"
      minimum_turning_radius: 0.40
      w_curve: 30.0                 # weight to minimize curvature of path
      w_dist: 0.0                   # weight to bind path to original as optional replacement for cost weight
      w_smooth: 15000.0             # weight to maximize smoothness of path
      w_cost: 0.015                 # weight to steer robot away from collision and cost
      w_cost_cusp: 0.04             # option to have higher weight during forward/reverse direction change which is often accompanied with dangerous rotations
      cusp_zone_length: 2.5         # length of the section around cusp in which nodes use w_cost_cusp instead of w_cost (-1.0 to disable w_cost_cusp)
      input_downsampling_factor: 3  # every n-th node of the path is taken. Useful for speed-up
      output_upsampling_factor: 1   # 0 - path remains downsampled, 1 - path is upsampled back to original granularity using cubic bezier, 2... - more upsampling

      # IMPORTANT: Although can be relevant for asymmetric robot footprints (e.g. diff drive with 2 actuated and 2 caster wheels),
      # this parameter introduces additional local minima to optimization function, making it harder to converge. If decided to be used,
      # then it should be done with caution and much higher max_iterations number should be considered.
      # (see test/test_costaware_smoother.cpp:testingObstacleAvoidanceNearCusps)
      # cost_check_points: [-0.185, 0.0, 1.0]  # points of robot footprint to grab costmap weight from. Format: [x1, y1, weight1, x2, y2, weight2, ...]

      optimizer:
        max_time: 10.0                # maximum compute time for smoother
        max_iterations: 70            # max iterations of smoother
        debug_optimizer: false        # print debug info
        gradient_tol: 5e3
        fn_tol: 1.0e-15
        param_tol: 1.0e-20
        advanced:
          min_line_search_step_size: 1.0e-9
          max_num_line_search_step_size_iterations: 20
          line_search_sufficient_function_decrease: 1.0e-3
          max_num_line_search_direction_restarts: 20
          max_line_search_step_expansion: 10
```