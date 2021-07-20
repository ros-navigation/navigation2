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
#include <vector>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_smac_planner/constants.hpp"

#ifndef NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
  : public nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::GridCollisionChecker
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute footprint
   * orientations for to speed up collision checking
   */
  GridCollisionChecker(
    nav2_costmap_2d::Costmap2D * costmap,
    unsigned int num_quantizations)
  : FootprintCollisionChecker(costmap),
    num_quantizations_(num_quantizations)
  {
  }

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(
    const nav2_costmap_2d::Footprint & footprint,
    const bool & radius,
    const double & possible_inscribed_cost)
  {
    possible_inscribed_cost_ = possible_inscribed_cost;
    footprint_is_radius_ = radius;

    // Use radius, no caching required
    if (radius) {
      return;
    }

    // No change, no updates required
    if (footprint == unoriented_footprint_) {
      return;
    }

    bin_size_ = 2.0 * M_PI / static_cast<double>(num_quantizations_);
    oriented_footprints_.reserve(num_quantizations_);
    double sin_th, cos_th;
    geometry_msgs::msg::Point new_pt;
    const unsigned int footprint_size = footprint.size();

    // Precompute the orientation bins for checking to use
    for (unsigned int i = 0; i != num_quantizations_; i++) {
      sin_th = sin(i * bin_size_);
      cos_th = cos(i * bin_size_);
      nav2_costmap_2d::Footprint oriented_footprint;
      oriented_footprint.reserve(footprint_size);

      for (unsigned int j = 0; j < footprint_size; j++) {
        new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
        new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
        oriented_footprint.push_back(new_pt);
      }

      oriented_footprints_.push_back(oriented_footprint);
    }

    unoriented_footprint_ = footprint;
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
      // if footprint, then we check for the footprint's points, but first see
      // if the robot is even potentially in an inscribed collision
      footprint_cost_ = costmap_->getCost(
        static_cast<unsigned int>(x), static_cast<unsigned int>(y));

      if (footprint_cost_ < possible_inscribed_cost_) {
        return false;
      }

      // If its inscribed, in collision, or unknown in the middle,
      // no need to even check the footprint, its invalid
      if (footprint_cost_ == UNKNOWN && !traverse_unknown) {
        return true;
      }

      if (footprint_cost_ == INSCRIBED || footprint_cost_ == OCCUPIED) {
        return true;
      }

      // if possible inscribed, need to check actual footprint pose.
      // Use precomputed oriented footprints are done on initialization,
      // offset by translation value to collision check
      int angle_bin = theta / bin_size_;
      geometry_msgs::msg::Point new_pt;
      const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
      nav2_costmap_2d::Footprint current_footprint;
      current_footprint.reserve(oriented_footprint.size());
      for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
        new_pt.x = wx + oriented_footprint[i].x;
        new_pt.y = wy + oriented_footprint[i].y;
        current_footprint.push_back(new_pt);
      }

      footprint_cost_ = footprintCost(current_footprint);

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
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const unsigned int & i,
    const bool & traverse_unknown)
  {
    footprint_cost_ = costmap_->getCost(i);
    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= INSCRIBED;
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
  std::vector<nav2_costmap_2d::Footprint> oriented_footprints_;
  nav2_costmap_2d::Footprint unoriented_footprint_;
  double footprint_cost_;
  bool footprint_is_radius_;
  unsigned int num_quantizations_;
  double bin_size_;
  double possible_inscribed_cost_{-1};
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
