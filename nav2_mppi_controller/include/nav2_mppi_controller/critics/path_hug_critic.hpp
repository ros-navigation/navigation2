// Copyright (c) 2025 Berkan Tali
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_

#include <vector>
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{
/**
 * @class mppi::critics::PathHugCritic
 * @brief Critic objective function for enforcing a hard corridor boundary
 * around the planned path. Intended as a stricter alternative to PathAlignCritic.
 * When use_soft_repulsion is false, trajectories inside the corridor pay zero cost
 * and any trajectory that exits is assigned collision_cost. When use_soft_repulsion
 * is true, a linear penalty ramp is applied between grace_distance and
 * max_allowed_distance with a hard veto at the boundary. If all trajectories
 * violate the boundary, graded fallback costs are applied to preserve optimizer
 * gradient for recovery.
 */
class PathHugCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;
  /**
   * @brief Evaluate cost related to corridor boundary enforcement
   *
   * @param data in/out critic data containing trajectories and path
   */
  void score(CriticData & data) override;

protected:
  /**
   * @brief Precompute cumulative arc-length distances along path segments
   *
   * @param path The current local path
   * @param num_segments Number of path segments to process
   */
  void updateCumulativeDistances(const models::Path & path, size_t num_segments);
  /**
   * @brief Compute squared distance from a point to the nearest path segment
   * within the arc-length search window around path_hint. Intended for
   * progress tracking, not global nearest-segment search.
   *
   * @param px X coordinate of the query point
   * @param py Y coordinate of the query point
   * @param path The current local path
   * @param num_segments Number of path segments
   * @param path_hint [in/out] Hint segment index, updated to closest found
   * @return Squared Euclidean distance to the nearest path segment
   */
  float computeMinDistToPathSq(
    float px, float py,
    const models::Path & path,
    size_t num_segments,
    Eigen::Index & path_hint);
  // cost_weight and cost_power only active in soft repulsion mode
  unsigned int power_{1};
  float weight_{10.0f};
  int trajectory_point_step_{4};
  float threshold_to_consider_{0.5f};
  float search_window_{0.15f};
  float lookahead_distance_{0.3f};
  float max_allowed_distance_{0.2f};
  float collision_cost_{100000.0f};
  bool use_soft_repulsion_{false};
  float grace_distance_{0.3f};
  std::vector<float> cumulative_distances_;
};
}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
