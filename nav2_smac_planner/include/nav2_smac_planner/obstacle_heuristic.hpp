// Copyright (c) 2026, Open Navigation LLC
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__OBSTACLE_HEURISTIC_HPP_
#define NAV2_SMAC_PLANNER__OBSTACLE_HEURISTIC_HPP_

#include <utility>
#include <vector>
#include <memory>
#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_smac_planner
{

typedef std::pair<float, uint64_t> ObstacleHeuristicElement;
struct ObstacleHeuristicComparator
{
  bool operator()(const ObstacleHeuristicElement & a, const ObstacleHeuristicElement & b) const
  {
    return a.first > b.first;
  }
};

typedef std::vector<ObstacleHeuristicElement> ObstacleHeuristicQueue;

/**
 * @class nav2_smac_planner::ObstacleHeuristic
 * @brief Obstacle Heuristic implementation for graph, Hybrid-A*
 */
class ObstacleHeuristic
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::ObstacleHeuristic
   */
  ObstacleHeuristic() {}

  /**
   * @brief A destructor for nav2_smac_planner::ObstacleHeuristic
   */
  ~ObstacleHeuristic() {}

  /**
   * @brief Compute the wavefront heuristic
   * @param costmap Costmap to use
   * @param goal_coords Coordinates to start heuristic expansion at
   */
  void resetObstacleHeuristic(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_i,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y,
    const bool downsample_obstacle_heuristic);

  /**
   * @brief Compute the Obstacle heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @return heuristic Heuristic value
   */
  float getObstacleHeuristic(
    const Coordinates & node_coords,
    const float & cost_penalty,
    const bool use_quadratic_cost_penalty,
    const bool downsample_obstacle_heuristic);

  inline float distanceHeuristic2D(
    const uint64_t idx, const unsigned int size_x,
    const unsigned int target_x, const unsigned int target_y)
  {
    int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
    int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
    return std::sqrt(dx * dx + dy * dy);
  }

  // ---- Incremental (LPA*) obstacle heuristic (opt-in) ----
  // An alternative to the lazy recompute that maintains a full goal-rooted cost
  // field and repairs it locally when the costmap changes, instead of wiping and
  // recomputing from scratch. Preserves the exact heuristic (bit-identical to a
  // full recompute) while being orders of magnitude cheaper for small changes —
  // the regime of high-frequency dynamic replanning. Uses its own tables and
  // does not touch the default lazy path above.

  /**
   * @brief Build the full incremental heuristic field from the goal.
   */
  void resetIncrementalObstacleHeuristic(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_i,
    const unsigned int & goal_x, const unsigned int & goal_y,
    const float & cost_penalty, const bool use_quadratic_cost_penalty);

  /**
   * @brief Repair the field for cells that changed since the last reset/update.
   * @return number of costmap cells that changed (0 == no work done)
   */
  unsigned int updateIncrementalObstacleHeuristic(
    const float & cost_penalty, const bool use_quadratic_cost_penalty);

  /**
   * @brief Look up the incremental heuristic value at a cell (raw field: +inf
   * for unreachable cells). getObstacleHeuristic() applies the inf->0 fallback
   * that keeps parity with the lazy path; this raw accessor is for the engine
   * and its tests.
   */
  float getIncrementalObstacleHeuristic(const Coordinates & node_coords);

  /**
   * @brief Select which field getObstacleHeuristic() reads from. Set once per
   * planning request from SearchInfo::incremental_obstacle_heuristic so a
   * runtime toggle can never leave a stale field selected.
   */
  void setIncrementalMode(const bool enabled) {inc_mode_ = enabled;}

  /**
   * @brief Whether an incremental field has already been built for a costmap of
   * the current size. When false the caller must reset (build) before updating.
   */
  bool isIncrementalFieldValid(
    const unsigned int & size_x, const unsigned int & size_y) const
  {
    return inc_size_x_ == size_x && inc_size_y_ == size_y &&
           inc_prev_cost_.size() == static_cast<std::size_t>(size_x) * size_y;
  }

protected:
  // Edge cost to ENTER cell `idx` (matches the lazy path's travel_cost model).
  float incrementalEnterCost(
    const unsigned int idx, const bool diagonal,
    const float & cost_penalty, const bool use_quadratic_cost_penalty) const;
  void incrementalUpdateVertex(
    const unsigned int idx, const float & cost_penalty,
    const bool use_quadratic_cost_penalty);
  void incrementalComputeShortestPath(
    const float & cost_penalty, const bool use_quadratic_cost_penalty);

  LookupTable obstacle_heuristic_lookup_table_;
  ObstacleHeuristicQueue obstacle_heuristic_queue_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;

  // Incremental-path state (separate from the lazy path above)
  std::vector<float> inc_g_;
  std::vector<float> inc_rhs_;
  std::vector<unsigned char> inc_prev_cost_;
  ObstacleHeuristicQueue inc_queue_;
  unsigned int inc_goal_index_{0};
  unsigned int inc_size_x_{0};
  unsigned int inc_size_y_{0};
  // When true, getObstacleHeuristic() reads the incremental field instead of
  // running the lazy dynamic-programming expansion.
  bool inc_mode_{false};
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__OBSTACLE_HEURISTIC_HPP_
