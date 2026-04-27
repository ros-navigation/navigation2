// Copyright (c) 2024 Nav2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <benchmark/benchmark.h>
#include <memory>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_dstar_lite_planner/dstar_lite.hpp"
#include "nav2_dstar_lite_planner/parameter_handler.hpp"

namespace
{

// Test subclass to expose protected DStarLite internals for benchmarking
class BenchmarkDStarLite : public nav2_dstar_lite_planner::DStarLite
{
public:
  explicit BenchmarkDStarLite(nav2_dstar_lite_planner::Parameters * params)
  : DStarLite(params) {}

  using nav2_dstar_lite_planner::DStarLite::initialize;
  using nav2_dstar_lite_planner::DStarLite::computeShortestPath;
  using nav2_dstar_lite_planner::DStarLite::updateVertex;
  using nav2_dstar_lite_planner::DStarLite::getChangedCells;
  using nav2_dstar_lite_planner::DStarLite::getPredecessors;
  using nav2_dstar_lite_planner::DStarLite::snapshotCostmap;
  using nav2_dstar_lite_planner::DStarLite::last_path_cost_;
  using nav2_dstar_lite_planner::DStarLite::replan_count_;

  void setupMap(int size_x, int size_y, double resolution,
                double origin_x, double origin_y)
  {
    costmap_ = new nav2_costmap_2d::Costmap2D(
      size_x, size_y, resolution, origin_x, origin_y, 0);
    size_x_ = size_x;
    size_y_ = size_y;
  }

  void setupStartGoal(double sx, double sy, double gx, double gy)
  {
    geometry_msgs::msg::PoseStamped start, goal;
    start.pose.position.x = sx;
    start.pose.position.y = sy;
    start.pose.orientation.w = 1.0;
    goal.pose.position.x = gx;
    goal.pose.position.y = gy;
    goal.pose.orientation.w = 1.0;
    clearStart();
    setStartAndGoal(start, goal);
  }

  bool runFullPlan(std::vector<nav2_dstar_lite_planner::WorldCoord> & path)
  {
    forceFullReplan();
    auto no_cancel = []() { return false; };
    return generatePath(path, no_cancel);
  }

  /// Run incremental replan after modifying costmap cells
  bool runIncrementalPlan(std::vector<nav2_dstar_lite_planner::WorldCoord> & path)
  {
    auto no_cancel = []() { return false; };
    auto changed = getChangedCells();
    for (const auto & cell : changed) {
      updateVertex(cell);
      auto preds = getPredecessors(cell);
      for (const auto & p : preds) {
        updateVertex(p);
      }
    }
    replan_count_++;
    nodes_opened = 0;
    computeShortestPath(no_cancel);

    auto & start_state = getOrCreateState(src_);
    if (start_state.rhs >= nav2_dstar_lite_planner::INF_COST) {
      path.clear();
      return false;
    }
    double cost = extractPath(path);
    if (path.empty()) { return false; }
    last_path_cost_ = cost;
    snapshotCostmap();
    return true;
  }

  ~BenchmarkDStarLite() { delete costmap_; }
};

// Global parameters
nav2_dstar_lite_planner::Parameters g_params;

}  // namespace

// ========== Scenario 1: Open Space First Plan ==========

static void BM_OpenSpace_FirstPlan(benchmark::State & state)
{
  rclcpp::init(0, nullptr);
  auto planner = std::make_unique<BenchmarkDStarLite>(&g_params);
  planner->setupMap(state.range(0), state.range(0), 1.0, 0.0, 0.0);
  int map_size = state.range(0);
  planner->setupStartGoal(1.0, 1.0,
    static_cast<double>(map_size) - 2.0,
    static_cast<double>(map_size) - 2.0);

  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  for (auto _ : state) {
    bool ok = planner->runFullPlan(path);
    if (!ok) { state.SkipWithError("No path found"); break; }
  }
  state.counters["nodes_opened"] = planner->nodes_opened;
  state.counters["path_length"] = static_cast<double>(path.size());
  rclcpp::shutdown();
}
BENCHMARK(BM_OpenSpace_FirstPlan)
  ->Args({100})
  ->Args({200})
  ->Args({400})
  ->Unit(benchmark::kMillisecond);

// ========== Scenario 2: Static Obstacles (20% random) ==========

static void BM_StaticObstacles_FirstPlan(benchmark::State & state)
{
  rclcpp::init(0, nullptr);
  int map_size = state.range(0);
  int obstacle_pct = state.range(1);

  auto planner = std::make_unique<BenchmarkDStarLite>(&g_params);
  planner->setupMap(map_size, map_size, 1.0, 0.0, 0.0);

  // Seed random obstacle placement
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist_x(0, map_size - 1);
  std::uniform_int_distribution<int> dist_y(0, map_size - 1);
  int num_obstacles = map_size * map_size * obstacle_pct / 100;

  for (int i = 0; i < num_obstacles; i++) {
    int x = dist_x(rng);
    int y = dist_y(rng);
    planner->costmap_->setCost(x, y, 254);
  }

  // Ensure start/goal are clear
  planner->costmap_->setCost(1, 1, 0);
  planner->costmap_->setCost(map_size - 2, map_size - 2, 0);
  planner->setupStartGoal(1.0, 1.0,
    static_cast<double>(map_size) - 2.0,
    static_cast<double>(map_size) - 2.0);

  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  for (auto _ : state) {
    bool ok = planner->runFullPlan(path);
    if (!ok) { state.SkipWithError("No path found"); break; }
  }
  state.counters["nodes_opened"] = planner->nodes_opened;
  state.counters["obstacle_pct"] = obstacle_pct;
  rclcpp::shutdown();
}
BENCHMARK(BM_StaticObstacles_FirstPlan)
  ->Args({200, 10})
  ->Args({200, 20})
  ->Args({200, 30})
  ->Unit(benchmark::kMillisecond);

// ========== Scenario 3: Incremental Replan (D* Lite's key advantage) ==========

static void BM_IncrementalReplan(benchmark::State & state)
{
  rclcpp::init(0, nullptr);
  int map_size = state.range(0);
  int num_changes = state.range(1);

  auto planner = std::make_unique<BenchmarkDStarLite>(&g_params);
  planner->setupMap(map_size, map_size, 1.0, 0.0, 0.0);
  planner->setupStartGoal(1.0, 1.0,
    static_cast<double>(map_size) - 2.0,
    static_cast<double>(map_size) - 2.0);

  // First: do a full plan to establish search state
  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  if (!planner->runFullPlan(path)) {
    state.SkipWithError("Initial plan failed");
    rclcpp::shutdown();
    return;
  }

  int64_t total_nodes = 0;
  for (auto _ : state) {
    // Restore previous snapshot so getChangedCells works correctly
    // (each iteration starts from the last snapshot state)
    // Add N random obstacles
    std::mt19937 rng(static_cast<unsigned int>(state.iterations()));
    for (int i = 0; i < num_changes; i++) {
      int x = std::uniform_int_distribution<int>(10, map_size - 10)(rng);
      int y = std::uniform_int_distribution<int>(10, map_size - 10)(rng);
      planner->costmap_->setCost(x, y, 254);
    }

    if (!planner->runIncrementalPlan(path)) {
      state.SkipWithError("Incremental plan failed");
      break;
    }
    total_nodes += planner->nodes_opened;
  }
  state.counters["avg_nodes_per_replan"] =
    static_cast<double>(total_nodes) / static_cast<double>(state.iterations());
  state.counters["num_costmap_changes"] = num_changes;
  rclcpp::shutdown();
}
BENCHMARK(BM_IncrementalReplan)
  ->Args({200, 10})
  ->Args({200, 50})
  ->Args({200, 100})
  ->Unit(benchmark::kMillisecond);

// ========== Scenario 4: Full Replan for Comparison ==========

static void BM_FullReplan_AfterChange(benchmark::State & state)
{
  rclcpp::init(0, nullptr);
  int map_size = state.range(0);
  int num_changes = state.range(1);

  auto planner = std::make_unique<BenchmarkDStarLite>(&g_params);
  planner->setupMap(map_size, map_size, 1.0, 0.0, 0.0);
  planner->setupStartGoal(1.0, 1.0,
    static_cast<double>(map_size) - 2.0,
    static_cast<double>(map_size) - 2.0);

  // First plan
  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  if (!planner->runFullPlan(path)) {
    state.SkipWithError("Initial plan failed");
    rclcpp::shutdown();
    return;
  }

  for (auto _ : state) {
    // Add N random obstacles
    std::mt19937 rng(static_cast<unsigned int>(state.iterations()));
    for (int i = 0; i < num_changes; i++) {
      int x = std::uniform_int_distribution<int>(10, map_size - 10)(rng);
      int y = std::uniform_int_distribution<int>(10, map_size - 10)(rng);
      planner->costmap_->setCost(x, y, 254);
    }
    // Full replan
    if (!planner->runFullPlan(path)) {
      state.SkipWithError("Full replan failed");
      break;
    }
  }
  state.counters["num_costmap_changes"] = num_changes;
  rclcpp::shutdown();
}
BENCHMARK(BM_FullReplan_AfterChange)
  ->Args({200, 10})
  ->Args({200, 50})
  ->Args({200, 100})
  ->Unit(benchmark::kMillisecond);
