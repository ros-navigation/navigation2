# Planners Smoothing Benchmark

This experiment runs a set with randomly generated goals for objective benchmarking.

Bechmarking scripts require the following python packages to be installed:

```
pip install transforms3d
pip install seaborn
pip install tabulate
```

To use the suite, modify the Nav2 bringup parameters `nav2_params.yaml` to include selected path planner:

```
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["SmacHybrid"]
    SmacHybrid:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.5
      motion_model_for_search: "DUBIN" # default, non-reverse motion
      smooth_path: false # should be disabled for experiment
      analytic_expansion_max_length: 0.3 # decreased to avoid robot jerking
```

... and path smoothers for benchmark:

```
 smoother_server:
   ros__parameters:
    smoother_plugins: ["simple_smoother", "constrained_smoother", "sg_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
    constrained_smoother:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      w_smooth: 100000.0 # tuned
    sg_smoother:
      plugin: "nav2_smoother::SavitzkyGolaySmoother"
```

Set global costmap, path planner and smoothers parameters to those desired in `nav2_params.yaml`.
Inside of `metrics.py`, you can change reference path planner / path smoothers to use.

For the benchmarking purposes, the clarification of execution time may be made for planner and smoother servers, to reduce impacts caused by other system actions outside of the planning / smoothing algorithm (optional):

```
diff --git a/nav2_planner/src/planner_server.cpp b/nav2_planner/src/planner_server.cpp
index c7a90bcb..6f93edbf 100644
--- a/nav2_planner/src/planner_server.cpp
+++ b/nav2_planner/src/planner_server.cpp
@@ -381,7 +381,10 @@ void PlannerServer::computePlanThroughPoses()
       }
 
       // Get plan from start -> goal
+      auto planning_start = steady_clock_.now();
       nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);
+      auto planning_duration = steady_clock_.now() - planning_start;
+      result->planning_time = planning_duration;
 
       if (!validatePath<ActionThroughPoses>(curr_goal, curr_path, goal->planner_id)) {
         throw nav2_core::NoValidPathCouldBeFound(goal->planner_id + "generated a empty path");
@@ -398,7 +401,7 @@ void PlannerServer::computePlanThroughPoses()
     publishPlan(result->path);
 
     auto cycle_duration = steady_clock_.now() - start_time;
-    result->planning_time = cycle_duration;
+    // result->planning_time = cycle_duration;
 
     if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
       RCLCPP_WARN(
diff --git a/nav2_smoother/src/nav2_smoother.cpp b/nav2_smoother/src/nav2_smoother.cpp
index ada1f664..610e9512 100644
--- a/nav2_smoother/src/nav2_smoother.cpp
+++ b/nav2_smoother/src/nav2_smoother.cpp
@@ -253,8 +253,6 @@ bool SmootherServer::findSmootherId(
 
 void SmootherServer::smoothPlan()
 {
-  auto start_time = steady_clock_.now();
-
   RCLCPP_INFO(get_logger(), "Received a path to smooth.");
 
   auto result = std::make_shared<Action::Result>();
@@ -271,6 +269,8 @@ void SmootherServer::smoothPlan()
     // Perform smoothing
     auto goal = action_server_->get_current_goal();
     result->path = goal->path;
+
+    auto start_time = steady_clock_.now();
     result->was_completed = smoothers_[current_smoother_]->smooth(
       result->path, goal->max_smoothing_duration);
     result->smoothing_duration = steady_clock_.now() - start_time;
```

Then execute the benchmarking:

- `ros2 launch ./smoother_benchmark_bringup.py` to launch the nav2 stack and path smoothers benchmarking
- `python3 ./process_data.py` to take the metric files and process them into key results (and plots)
