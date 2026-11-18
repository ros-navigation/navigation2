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
#include "nav2_smac_planner/collision_checker.hpp"
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
   * @param collision_checker Collision checker for the costmap and max traversal cost
   * @param goal_coords Coordinates to start heuristic expansion at
   * @param traverse_unknown Whether unknown space is traversable
   */
  void resetObstacleHeuristic(
    GridCollisionChecker * collision_checker,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y,
    const bool downsample_obstacle_heuristic,
    const bool traverse_unknown);

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

protected:
  /**
   * @brief Whether a heuristic cell is traversable, if any of its costmap cells are valid
   * @param idx Heuristic cell index
   * @param size_x Size of the heuristic grid in X
   * @param scale Number of costmap cells per heuristic cell along each axis
   * @param cost Lowest valid cost of its costmap cells
   * @return If the cell is traversable
   */
  bool isCellOpen(
    const unsigned int idx, const unsigned int size_x, const unsigned int scale,
    float & cost) const;

  LookupTable obstacle_heuristic_lookup_table_;
  ObstacleHeuristicQueue obstacle_heuristic_queue_;
  GridCollisionChecker * collision_checker_{nullptr};
  bool traverse_unknown_{true};
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__OBSTACLE_HEURISTIC_HPP_
