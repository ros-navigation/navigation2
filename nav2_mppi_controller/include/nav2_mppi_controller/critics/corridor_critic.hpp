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
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CORRIDOR_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CORRIDOR_CRITIC_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/safe_corridor.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CorridorCritic
 * @brief Penalizes MPPI trajectory samples that deviate beyond the safe corridor
 * around the reference path.
 *
 * The safe corridor at each path point is defined as a circle of radius
 *   corridor_width[k] = clamp(esdf(path[k]) - robot_radius, 0, max_corridor_width)
 * centered on path point k. Any trajectory point whose distance to the nearest
 * path point exceeds that path point's corridor width incurs a penalty
 * proportional to the excess deviation.
 *
 * This complements EsdfCritic (which punishes proximity to obstacles) by also
 * penalizing trajectories that wander into narrow regions or dead-ends that the
 * path does not cross, even if those regions are currently obstacle-free.
 *
 * Algorithm per scoring cycle:
 *   1. Compute corridor_width[k] for every path point k (O(path_len) ESDF lookups).
 *   2. For each trajectory time step j, find the representative path point by
 *      matching the mean trajectory position at step j (O(path_len) per step).
 *   3. For every trajectory sample i at step j, add a cost proportional to
 *      max(0, dist(traj[i,j], path[nk]) - corridor_width[nk]).
 *      This is vectorized over i using Eigen.
 *
 * Parameters (ROS2 params, namespace = critic name):
 *   enabled              (bool,  default true)
 *   cost_power           (uint,  default 1)
 *   cost_weight          (float, default 5.0)
 *   robot_radius         (float, default 0.0 — auto from inscribed radius)
 *   max_corridor_width   (float, default 3.0 m)
 *   threshold_to_consider (float, default 0.5 m — skip critic near goal)
 *   inflation_layer_name (string, default "" — auto-detect)
 */
class CorridorCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

protected:
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer_{nullptr};

  float robot_radius_{0.0f};
  float max_corridor_width_{3.0f};
  float weight_{5.0f};
  float threshold_to_consider_{0.5f};
  std::string inflation_layer_name_;
  unsigned int power_{1};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CORRIDOR_CRITIC_HPP_
