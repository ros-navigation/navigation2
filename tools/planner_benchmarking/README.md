# Planning Benchmark

This experiment runs a set of planners over randomly generated maps, with randomly generated goals for objective benchmarking.

To use, modify the Nav2 bringup parameters to include the planners of interest:

```
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["SmacHybrid", "Smac2d", "SmacLattice", "Navfn", "ThetaStar"]
    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
    Smac2d:
      plugin: "nav2_smac_planner/SmacPlanner2D"
    SmacLattice:
      plugin: "nav2_smac_planner/SmacPlannerLattice"
    Navfn:
      plugin: "nav2_navfn_planner/NavfnPlanner"
    ThetaStar:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner"
```

Set global costmap settings to those desired for benchmarking. The global map will be automatically set in the script. Inside of `metrics.py`, you can modify the map or set of planners to use.

Launch the benchmark via `ros2 launch ./planning_benchmark_bringup.py` to launch the planner and map servers, then run each script in this directory:

- `metrics.py` to capture data in `.pickle` files.
- `process_data.py` to take the metric files and process them into key results (and plots)
