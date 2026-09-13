// Copyright (c) 2026
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__DYN_FOOTPRINT_COST_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__DYN_FOOTPRINT_COST_CRITIC_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/polygon_utils/velocity_polygon.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::DynFootprintCostCritic
 * @brief Critic objective function for obstacle avoidance using a
 * velocity-scaled footprint along the rollout horizon. Always operates in
 * footprint mode (the costmap must be configured with an explicit
 * 'footprint', not 'robot_radius') since the whole point is to scale that
 * footprint's polygon with each pose's sampled speed.
 */
class DynFootprintCostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param data [in, out] critic data to score
   */
  void score(CriticData & data) override;

private:
  /**
    * @brief An implementation of worldToMap fully using floats
    * @param wx Float world X coord
    * @param wy Float world Y coord
    * @param mx unsigned int map X coord
    * @param my unsigned into map Y coord
    * @return if successful
    */
  inline bool worldToMapFloat(float wx, float wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < origin_x_ || wy < origin_y_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_) {
      return true;
    }
    return false;
  }

  /**
    * @brief A local implementation of getIndex
    * @param mx unsigned int map X coord
    * @param my unsigned into map Y coord
    * @return Index
    */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * size_x_ + mx;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_{nullptr};

  bool is_tracking_unknown_{true};
  float collision_cost_{0.0f};
  float critical_cost_{0.0f};
  unsigned int near_collision_cost_{253};
  float weight_{0};
  unsigned int trajectory_point_step_;

  float origin_x_, origin_y_, resolution_;
  unsigned int size_x_, size_y_;

  float near_goal_distance_;
  std::string inflation_layer_name_;

  unsigned int power_{0};
  std::shared_ptr<mppi::polygon_utils::VelocityPolygon> velocity_polygon_{nullptr};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__DYN_FOOTPRINT_COST_CRITIC_HPP_
