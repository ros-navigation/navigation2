# Planners Smoothing Benchmark

This experiment runs a set with randomly generated goals for objective benchmarking.

Bechmarking scripts require the following python packages to be installed:

```
pip install transforms3d
pip install seaborn
pip install tabulate
```

To perform the benchmarking, the following changes should be made for planner and smoother servers:

```
diff --git a/nav2_planner/src/planner_server.cpp b/nav2_planner/src/planner_server.cpp
index c7a90bcb..510bb6ea 100644
--- a/nav2_planner/src/planner_server.cpp
+++ b/nav2_planner/src/planner_server.cpp
@@ -285,7 +285,7 @@ bool PlannerServer::getStartPose(
   typename std::shared_ptr<const typename T::Goal> goal,
   geometry_msgs::msg::PoseStamped & start)
 {
-  if (goal->use_start) {
+  if (true) {
     start = goal->start;
   } else if (!costmap_ros_->getRobotPose(start)) {
     return false;
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

To use the suite, modify the Nav2 bringup parameters to include selected path planner:

```
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["SmacHybrid"]
    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.5
      motion_model_for_search: "DUBIN" # default, non-reverse motion
      smooth_path: false # should be disabled for experiment
      analytic_expansion_max_length: 0.3 # decreased to avoid robot jerking
```

... and path smoothers for benchmark:

```
 smoother_server:
   ros__parameters:
    smoother_plugins: ["simple_smoother", "constrained_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      holonomic: False # should be disabled to utilize motion/rotation model
    constrained_smoother:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      w_smooth: 100000.0 # tuned
```

Then enable SmoothPath sequence for selected path planner, as shown for default `navigate_to_pose_w_replanning_and_recovery.xml` behavior tree:

```
       <PipelineSequence name="NavigateWithReplanning">
         <RateController hz="1.0">
           <RecoveryNode number_of_retries="1" name="ComputePathToPose">
-            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
+            <Sequence name="ComputeAndSmoothPath">
+              <ComputePathToPose goal="{goal}" path="{path}" planner_id="SmacHybrid"/>
+              <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}"/>
+            </Sequence>
             <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
           </RecoveryNode>
         </RateController>
```

Inside of `metrics.py`, you can modify the map or reference planner / path smoothers to use. Set global costmap, path planner and smoothers settings to those desired for benchmarking and execute it:

- `ros2 launch ./smoother_benchmark_bringup.py` to launch the nav2 stack and path smoothers benchmarking
- `python3 ./process_data.py` to take the metric files and process them into key results (and plots)
