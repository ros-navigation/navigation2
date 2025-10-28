// Copyright (c) 2025, Berkan Tali
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may not use a copy of the License at
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

#include <Eigen/Dense>
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::PathHugCritic
 * @brief Critic plugin for penalizing trajectories that deviate from the global path.
 *
 * This critic calculates a cost for each trajectory based on its distance from the
 * reference path. The cost is an accumulation of the minimum distances from sampled points
 * along the trajectory to the reference path. This encourages the controller to generate
 * trajectories that "hug" or closely follow the provided global plan.
 */
class PathHugCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize the critic, loading parameters.
   */
  void initialize() override;

  /**
   * @brief Evaluate the critic score for all trajectories.
   * @param data The critic data object containing information about the trajectories, path, and costs.
   */
  void score(CriticData & data) override;

private:
  void updatePathCache(const models::Path & path);

  void computeDistancesToPathVectorized(
    const Eigen::ArrayXf & points_x,
    const Eigen::ArrayXf & points_y,
    Eigen::ArrayXf & distances);

  // Parameters
  unsigned int power_;
  float weight_;
  double search_window_;
  int sample_stride_;

  // Path cache for efficient computation
  size_t path_size_cache_{0};
  Eigen::ArrayXf path_x_cache_;
  Eigen::ArrayXf path_y_cache_;
  Eigen::ArrayXf segment_lengths_;
  Eigen::ArrayXf cumulative_distances_;
  Eigen::ArrayXf segment_dx_;
  Eigen::ArrayXf segment_dy_;
  Eigen::ArrayXf segment_len_sq_;

  // Per-evaluation cache
  Eigen::ArrayXf cost_array_;
  Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> closest_indices_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
