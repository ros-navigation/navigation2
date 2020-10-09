// Copyright (c) 2020, Samsung Research America
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

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "smac_planner/constants.hpp"

#ifndef SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define SMAC_PLANNER__COLLISION_CHECKER_HPP_

namespace smac_planner
{

/**
 * @class smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
  : public nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
{
public:
  /**
   * @brief A constructor for smac_planner::GridCollisionChecker
   * @param costmap The costmap to collision check against
   */
  GridCollisionChecker(
    nav2_costmap_2d::Costmap2D * costmap)
  : FootprintCollisionChecker(costmap)
  {
  }

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(const nav2_costmap_2d::Footprint & footprint, const bool & radius)
  {
    unoriented_footprint_ = footprint;
    footprint_is_radius_ = radius;
  }

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle of pose to check against
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown)
  {
    // Assumes setFootprint already set
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);

    if (!footprint_is_radius_) {
      // if footprint, then we check for the footprint's points
      footprint_cost_ = footprintCostAtPose(
        wx, wy, static_cast<double>(theta), unoriented_footprint_);
      if (footprint_cost_ == UNKNOWN && traverse_unknown) {
        return false;
      }

      // if occupied or unknown and not to traverse unknown space
      return footprint_cost_ >= OCCUPIED;
    } else {
      // if radius, then we can check the center of the cost assuming inflation is used
      footprint_cost_ = costmap_->getCost(
        static_cast<unsigned int>(x), static_cast<unsigned int>(y));

      if (footprint_cost_ == UNKNOWN && traverse_unknown) {
        return false;
      }

      // if occupied or unknown and not to traverse unknown space
      return footprint_cost_ >= INSCRIBED;
    }
  }

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost()
  {
    // Assumes inCollision called prior
    return static_cast<float>(footprint_cost_);
  }

protected:
  nav2_costmap_2d::Footprint unoriented_footprint_;
  double footprint_cost_;
  bool footprint_is_radius_;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__COLLISION_CHECKER_HPP_
