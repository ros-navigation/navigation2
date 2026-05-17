// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__ESDF_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__ESDF_CRITIC_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::EsdfCritic
 * @brief Critic that scores trajectories using exact Euclidean distances from
 * the ESDF exposed by InflationLayer. This avoids the lossy log-inversion
 * approximation used by ObstaclesCritic when back-computing distance from
 * costmap costs, yielding more accurate repulsion and margin scoring.
 *
 * Requires the costmap to have an InflationLayer (not LegacyInflationLayer).
 */
class EsdfCritic : public CriticFunction
{
public:
  void initialize() override;

  void score(CriticData & data) override;

protected:
  /**
   * @brief Check if a pose is in collision using exact ESDF distance.
   * @param esdf_dist  Euclidean distance (m) to nearest obstacle from the cell
   * @param x          World x of the pose (for footprint check)
   * @param y          World y of the pose
   * @param theta      Heading of the pose
   * @return true if the pose is in collision
   */
  inline bool inCollision(float esdf_dist, float x, float y, float theta);

  /**
   * @brief Fast float world-to-map conversion (avoids double round-trip).
   */
  inline bool worldToMapFloat(float wx, float wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < origin_x_ || wy < origin_y_) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    return mx < size_x_ && my < size_y_;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};

  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer_{nullptr};

  bool consider_footprint_{false};
  bool is_tracking_unknown_{true};

  float collision_cost_{100000.0f};
  float collision_margin_distance_{0.10f};
  float near_goal_distance_{0.5f};

  float inscribed_radius_{0.0f};
  float circumscribed_radius_{0.0f};
  float inflation_radius_{0.0f};

  float repulsion_weight_{1.5f};
  float critical_weight_{20.0f};

  float origin_x_{0.0f}, origin_y_{0.0f}, resolution_{0.0f};
  unsigned int size_x_{0}, size_y_{0};

  std::string inflation_layer_name_;
  unsigned int power_{1};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__ESDF_CRITIC_HPP_
