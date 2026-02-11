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

#ifndef NAV2_SMAC_PLANNER__DISTANCE_HEURISTIC_HPP_
#define NAV2_SMAC_PLANNER__DISTANCE_HEURISTIC_HPP_

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"

namespace nav2_smac_planner
{
struct HybridMotionTable;
struct LatticeMotionTable;
class NodeHybrid;
class NodeLattice;

/**
 * @class nav2_smac_planner::DistanceHeuristic
 * @brief Distance Heuristic implementation for graph, Hybrid-A*
 */
template<typename NodeT>
class DistanceHeuristic
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::DistanceHeuristic
   */
  DistanceHeuristic() {}

  /**
   * @brief Compute the SE2 distance heuristic
   * @param lookup_table_dim Size, in costmap pixels, of the
   * each lookup table dimension to populate
   * @param motion_model Motion model to use for state space
   * @param dim_3_size Number of quantization bins for caching
   * @param search_info Info containing minimum radius to use
   */
  template<typename MotionTableT>
  void precomputeDistanceHeuristic(
    const float & lookup_table_dim,
    const MotionModel & motion_model,
    const unsigned int & dim_3_size,
    const SearchInfo & search_info,
    MotionTableT & motion_table);

  /**
   * @brief Compute the Distance heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @param obstacle_heuristic Value of the obstacle heuristic to compute
   * additional motion heuristics if required
   * @return heuristic Heuristic value
   */
  template<typename MotionTableT>
  float getDistanceHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const float & obstacle_heuristic,
    MotionTableT & motion_table);

protected:
  // Dubin / Reeds-Shepp lookup and size for dereferencing
  LookupTable dist_heuristic_lookup_table_;
  float size_lookup_;
};

}  // namespace nav2_smac_planner
#endif  // NAV2_SMAC_PLANNER__DISTANCE_HEURISTIC_HPP_
