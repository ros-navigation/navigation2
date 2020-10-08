# Smac Planner

The SmacPlanner is a plugin for the Nav2 Planner server. It includes currently 2 distinct plugins:
- `SmacPlanner`: a highly optimized fully reconfigurable Hybrid-A* implementation supporting Dubin and Reeds-Shepp models.
- `SmacPlanner2D`: a highly optimized fully reconfigurable grid-based A* implementation supporting Moore and Von Neumann models.

It also introduces the following basic building blocks:
- `CostmapDownsampler`: A library to take in a costmap object and downsample it to another resolution.
- `AStar`: A generic and highly optimized A* template library used by the planning plugins to search. Template implementations are provided for grid-A* and SE2 Hybrid-A* planning. Additional template for 3D planning also could be made available.
- `CollisionChecker`: Collision check based on a robot's radius or footprint.
- `Smoother`: A Conjugate-gradient (CG) smoother with several optional cost function implementations for use. This is a cost-aware smoother unlike b-splines or bezier curves.

We have users reporting using this on:
- Delivery robots
- Industrial robots

## Introduction

The `smac_planner` package contains an optimized templated A* search algorithm used to create multiple A\*-based planners for multiple types of robot platforms. It was built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research](https://www.sra.samsung.com/). We support differential-drive and omni-directional drive robots using the `SmacPlanner2D` planner which implements a cost-aware A\* planner. We support cars, car-like, and ackermann vehicles using the `SmacPlanner` plugin which implements a Hybrid-A\* planner. This plugin is also useful for curvature constrained planning, like when planning robot at high speeds to make sure they don't flip over or otherwise skid out of control. It is also applicable to non-round robots (such as large rectangular or arbitrary shaped robots of differential/omnidirectional drivetrains) that need pose-based collision checking.

The `SmacPlanner` fully-implements the Hybrid-A* planner as proposed in [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf), including hybrid searching, CG smoothing, analytic expansions and hueristic functions.

In summary...

The `SmacPlanner` is designed to work with:
- Ackermann, car, and car-like robots
- High speed or curvature constrained robots (as to not flip over, skid, or dump load at high speeds)
- Arbitrary shaped, non-circular differential or omnidirectional robots requring SE2 collision checking

The `SmacPlanner2D` is designed to work with:
- Circular, differential or omnidirectional robots
- Relatively small robots with respect to environment size (e.g. RC car in a hallway or large robot in a convention center) that can be approximated by circular footprint.

## Features 

We further improve on the Hybrid-A\* work in the following ways:
- Remove need for upsampling by searching with 10x smaller motion primitives (same as their upsampling ratio).
- Multi-resolution search allowing planning to occur at a coarser resolution for wider spaces (O(N^2) faster).
- Cost-aware penalty functions in search resulting in far smoother plans (further reducing requirement to smooth).
- Additional cost functions in the CG smoother including path deflection.
- Approximated smoother Voronoi field using costmap inflation function.
- Fixed 3 mathematical issues in the original paper resulting in higher quality smoothing.
- Faster planning than original paper by highly optimizing the template A\* algorithm.
- Automatically adjusted search motion model sizes by motion model, costmap resolution, and bin sizing.
- Closest path on approach within tolerance if exact path cannot be found or in invalid space.
- Multi-model hybrid searching including Dubin and Reeds-Shepp models. More models may be trivially added.
- Time monitoring of planning to dynamically scale the maximum CG smoothing time based on remaining planning duration requested. 
- High unit and integration test coverage, doxygen documentation.
- Uses modern C++14 language features and individual components are easily reusable.
- Speed optimizations: Fast approximation of shortest paths with wavefront hueristic, no data structure graph lookups in main loop, near-zero copy main loop, Voronoi field approximation, et al.
- Templated Nodes and A\* implementation to support additional robot extensions.

All of these features (multi-resolution, models, smoother, etc) are also available in the 2D `SmacPlanner2D` plugin.

The 2D A\* implementation also does not have any of the weird artifacts introduced by the gradient wavefront-based 2D A\* implementation in the NavFn Planner. While this 2D A\* planner is slightly slower, I believe it's well worth the increased quality in paths. Though the `SmacPlanner2D` is grid-based, any reasonable local trajectory planner - including those supported by Navigation2 - will not have any issue with grid-based plans.

## Metrics

The original Hybrid-A\* implementation boasted planning times of 50-300ms for planning across 102,400 cell maps with 72 angular bins. We see much faster results in our evaluations:

- **2-20ms** for planning across 147,456 (1.4x larger) cell maps with 72 angular bins.
- **30-120ms** for planning across 344,128 (3.3x larger) cell map with 72 angular bins.

For example, the following path (roughly 85 meters) path took 33ms to compute.

![alt text](test/path.png)

## Design

The basic design centralizes a templated A\* implementation that handles the search of a graph of nodes. The implementation is templated by the nodes, `NodeT`, which contain the methods needed to compute the hueristics, travel costs, and search neighborhoods. The outcome of this design is then a standard A\* implementation that can be used to traverse any type of graph as long as a node template can be created for it.

We provide by default a 2D node template (`Node2D`) which does 2D grid-search with either 4 or 8-connected neighborhoods, but the smoother can be used to smooth it out. We also provide a SE2 node template (`NodeSE2`) which does SE2 (X, Y, theta) search and collision checking on Dubin or Reeds-Shepp motion models. Additional templates could be easily made and included for 3D grid search and non-grid base searching like routing.

Additional node templates could be added into the future to better support other types of robot path planning, such as including a state lattice motion primitive node and 3D path planning. Pull requests or discussions aroudn this point is encouraged.

In the ROS2 facing plugin, we take in the global goals and pre-process the data to feed into the templated A\* used. This includes processing any requests to downsample the costmap to another resolution to speed up search and smoothing the resulting A\* path. For the SE2 and `SmacPlanner` plugins, the path is promised to be kinematically feasible due to the kinematically valid models used in branching search. The 2D A\* is also promised to be feasible for differential and omni-directional robots.

We isolated the A\*, costmap downsampler, smoother, and Node template objects from ROS2 to allow them to be easily testable independently of ROS or the planner. The only place ROS is used is in the planner plugins themselves. 

## Parameters

See inline description of parameters in the `SmacPlanner`. This includes comments as specific parameters apply to `SmacPlanner2D` and `SmacPlanner` in SE2 place.

```
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: True

    GridBased:
      plugin: "smac_planner/SmacPlanner"
      tolerance: 0.5                    # tolerance for planning if unable to reach exact pose, in meters, for 2D node
      downsample_costmap: false         # whether or not to downsample the map
      downsampling_factor: 1            # multiplier for the resolution of the costmap layer (e.g. 2 on a 5cm costmap would be 10cm)
      allow_unknown: false              # allow traveling in unknown space
      max_iterations: -1                # maximum total iterations to search for before failing
      max_on_approach_iterations: 1000  # maximum number of iterations to attempt to reach goal once in tolerance, 2D only
      max_planning_time_ms: 2000.0      # max time in ms for planner to plan, smooth, and upsample. Will scale maximum smoothing and upsampling times based on remaining time after planning.
      smooth_path: false                # Whether to smooth searched path
      motion_model_for_search: "DUBIN"  # 2D Moore, Von Neumann; SE2 Dubin, Redds-Shepp
      angle_quantization_bins: 72       # For SE2 node: Number of angle bins for search, must be 1 for 2D node (no angle search)
      minimum_turning_radius: 0.20      # For SE2 node & smoother: minimum turning radius in m of path / vehicle
      reverse_penalty: 2.1              # For Reeds-Shepp model: penalty to apply if motion is reversing, must be => 1
      change_penalty: 0.20              # For SE2 node: penalty to apply if motion is changing directions, must be >= 0
      non_straight_penalty: 1.05        # For SE2 node: penalty to apply if motion is non-straight, must be => 1
      cost_penalty: 1.3                 # For SE2 node: penalty to apply to higher cost zones

      smoother:
        smoother:
          w_curve: 30.0                 # weight to minimize curvature of path
          w_dist: 0.0                   # weight to bind path to original as optional replacement for cost weight
          w_smooth: 30000.0             # weight to maximize smoothness of path
          w_cost: 0.025                 # weight to steer robot away from collision and cost
          cost_scaling_factor: 10.0     # this should match the inflation layer's parameter

        # I do not recommend users mess with this unless they're doing production tuning
        optimizer:
          max_time: 0.10                # maximum compute time for smoother
          max_iterations: 500           # max iterations of smoother
          debug_optimizer: false        # print debug info
          gradient_tol: 1.0e-10
          fn_tol: 1.0e-20
          param_tol: 1.0e-15
          advanced:
            min_line_search_step_size: 1.0e-20
            max_num_line_search_step_size_iterations: 50
            line_search_sufficient_function_decrease: 1.0e-20
            max_num_line_search_direction_restarts: 10
            max_line_search_step_expansion: 50
```

## Topics

| Topic           | Type              |
|-----------------|-------------------|
| unsmoothed_path | nav_msgs/Path     |


## Install

```
sudo apt-get install ros-<ros2-distro>-smac-planner
```

## Etc (Important Side Notes)

### Potential Fields

Many users and default navigation configuration files I find are really missing the point of the inflation layer. While it's true that you can simply inflate a small radius around the walls, the _true_ value of the inflation layer is creating a consistent potential field around the entire map. 

Some of the most popular tuning guides for Navigation / Navigation2 even [call this out specifically](https://arxiv.org/pdf/1706.09068.pdf) that there's substantial benefit to creating a gentle potential field across the width of the map - after inscribed costs are applied - yet very few users do this. 

This habit actually results in paths produced by NavFn, Global Planner, and now SmacPlanner to be very suboptimal. They really want to look for a smooth potential field rather than wide open 0-cost spaces in order to stay in the middle of spaces and deal with close-by moving obstacles better.

So it is my recommendation in using this package, as well as all other cost-aware search planners available in ROS, to increase your inflation layer cost scale in order to adequately produce a smooth potential across the entire map. For very large open spaces, its fine to have 0-cost areas in the middle, but for halls, aisles, and similar; **please create a smooth potential to provide the best performance**. 

### 2D Search and Smoothing

While the 2D planner has the smoother available (albeit, default parameters are tuned for the Hybrid-A\* planner, so you may need to play with that), my recommendation is not to use it.

The 2D planner provides a 4-connected or 8-connected neighborhood path. This path may have little zig-zags in order to get at another non-90 or non-45 degree heading. That is totally fine. Your local trajectory planner such as DWB and TEB take these points into account to follow, but you won't see any zig-zag behaviors of your final robot motion after given to a trajectory planner.

The smoothing is more "pleasing" to human eyes, but you don't want to be owning additional compute when it doesn't largely impact the output. However, if you have a more sensitive local trajectory planner like a carrot follower (e.g. pure pursuit), then you will want to smooth out the paths in order to have something more easily followable.

Take this advise into account. Some good numbers to potentially start with would be `cost_scaling_factor: 10.0` and `inflation_radius: 5.5`.

### Costmap Resolutions

We provide for both the Hybrid-A\* and 2D A\* implementations a costmap downsampler option. This can be **incredible** beneficial when planning very long paths in larger spaces. The motion models for SE2 planning and neighborhood search in 2D planning is proportional to the costmap resolution. By downsampling it, you can N^2 reduce the number of expansions required to achieve a particular goal. However, the lower the resolution, the larger small obstacles appear and you won't be able to get super close to obstacles. This is a trade-off to make and test. Some numbers I've seen are 2-4x drops in planning CPU time for a 2-3x downsample rate. For 60m paths in an office space, I was able to get it << 100ms at a 2-3x downsample rate.

I recommend users using a 5cm resolution costmap and playing with the different values of downsampling rate until they achieve what they think is optimal performance (lowest number of expansions vs. necessity to achieve fine goal poses). Then, I would recommend to change the global costmap resolution to this new value. That way you don't own the compute of downsampling and maintaining a higher-resolution costmap that isn't used.

Remember, the global costmap is **only** there to provide an environment for the planner to work in. It is not there for human-viewing even if a more fine resolution costmap is more human "pleasing". If you use multiple planners in the planner server, then you will want to use the highest resolution for the most needed planner and then use the downsampler to downsample to the Hybrid-A* resolution. 
