# Constrained Smoother

A smoother plugin for `nav2_smoother` based on the original deprecated smoother in `nav2_smac_planner` by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) and put into operational state by [**RoboTech Vision**](https://robotechvision.com/). Suitable for applications which need planned global path to be pushed away from obstacles and/or for Reeds-Shepp motion models.

See documentation on docs.nav2.org: https://docs.nav2.org/configuration/packages/configuring-constrained-smoother.html


Example of configuration (see indoor_navigation package of this repo for a full launch configuration):

```
smoother_server:
  ros__parameters:
    smoother_plugins: ["SmoothPath"]

    SmoothPath:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      reversing_enabled: true       # whether to detect forward/reverse direction and cusps. Should be set to false for paths without orientations assigned
      path_downsampling_factor: 3   # every n-th node of the path is taken. Useful for speed-up
      path_upsampling_factor: 1     # 0 - path remains downsampled, 1 - path is upsampled back to original granularity using cubic bezier, 2... - more upsampling
      keep_start_orientation: true  # whether to prevent the start orientation from being smoothed
      keep_goal_orientation: true   # whether to prevent the gpal orientation from being smoothed
      minimum_turning_radius: 0.40  # minimum turning radius the robot can perform. Can be set to 0.0 (or w_curve can be set to 0.0 with the same effect) for diff-drive/holonomic robots
      w_curve: 30.0                 # weight to enforce minimum_turning_radius
      w_dist: 0.0                   # weight to bind path to original as optional replacement for cost weight
      w_smooth: 2000000.0           # weight to maximize smoothness of path
      w_cost: 0.015                 # weight to steer robot away from collision and cost

      # Parameters used to improve obstacle avoidance near cusps (forward/reverse movement changes)
      # See the [docs page](https://docs.nav2.org/configuration/packages/configuring-constrained-smoother) for further clarification
      w_cost_cusp_multiplier: 3.0   # option to have higher weight during forward/reverse direction change which is often accompanied with dangerous rotations
      cusp_zone_length: 2.5         # length of the section around cusp in which nodes use w_cost_cusp_multiplier (w_cost rises gradually inside the zone towards the cusp point, whose costmap weight equals w_cost*w_cost_cusp_multiplier)

      # Points in robot frame to grab costmap values from. Format: [x1, y1, weight1, x2, y2, weight2, ...]
      # IMPORTANT: Requires much higher number of iterations to actually improve the path. Uncomment only if you really need it (highly elongated/asymmetric robots)
      # See the [docs page](https://docs.nav2.org/configuration/packages/configuring-constrained-smoother) for further clarification
      # cost_check_points: [-0.185, 0.0, 1.0]

      optimizer:
        max_iterations: 70            # max iterations of smoother
        debug_optimizer: false        # print debug info
        gradient_tol: 5e3
        fn_tol: 1.0e-15
        param_tol: 1.0e-20
```

Note: Smoothing paths which contain multiple subsequent poses at one point (e.g. in-place rotations from Smac lattice planners) is currently not supported

Note: Constrained Smoother is recommended to be used on a path with a bounded length. TruncatePathLocal BT Node can be used for extracting a relevant path section around robot (in combination with DistanceController to achieve periodicity)
