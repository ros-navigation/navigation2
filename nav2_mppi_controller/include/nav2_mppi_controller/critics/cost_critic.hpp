// Copyright (c) 2023 Robocc Brice Renaudeau
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_

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
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 */
class CostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
  inline bool inCollision(float cost, float x, float y, float theta)
  {
    // If consider_footprint_ check footprint scort for collision
    float score_cost = cost;
    if (consider_footprint_ &&
      (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f))
    {
      score_cost = static_cast<float>(collision_checker_.footprintCostAtPose(
          static_cast<double>(x), static_cast<double>(y), static_cast<double>(theta),
          costmap_ros_->getRobotFootprint()));
    }

    switch (static_cast<unsigned char>(score_cost)) {
      case (nav2_costmap_2d::LETHAL_OBSTACLE):
        return true;
      case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
        return consider_footprint_ ? false : true;
      case (nav2_costmap_2d::NO_INFORMATION):
        return is_tracking_unknown_ ? false : true;
    }

    return false;
  }

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  inline float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

  /**
    * @brief An implementation of worldToMap fully using floats
    * @param wx Float world X coord
    * @param wy Float world Y coord
    * @param mx unsigned int map X coord
    * @param my unsigned into map Y coord
    * @return if successsful
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
  float possible_collision_cost_;

  bool consider_footprint_{true};
  bool is_tracking_unknown_{true};
  float circumscribed_radius_{0.0f};
  float circumscribed_cost_{0.0f};
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
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
