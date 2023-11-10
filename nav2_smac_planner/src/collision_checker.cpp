// Copyright (c) 2021, Samsung Research America
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

#include "nav2_smac_planner/collision_checker.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  nav2_costmap_2d::Costmap2D * costmap,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_inscribed_cost)
{
  possible_inscribed_cost_ = static_cast<float>(possible_inscribed_cost);
  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
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

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return false;
  }

  // Assumes setFootprint already set
  double wx, wy;
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    footprint_cost_ = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x + 0.5), static_cast<unsigned int>(y + 0.5)));

    if (footprint_cost_ < possible_inscribed_cost_) {
      if (possible_inscribed_cost_ > 0) {
        return false;
      } else {
        RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 1000,
          "Inflation layer either not found or inflation is not set sufficiently for "
          "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
          " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
          "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
          " for full instructions. This will substantially impact run-time performance.");
      }
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
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    footprint_cost_ = static_cast<float>(footprintCost(current_footprint));

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= OCCUPIED;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    footprint_cost_ = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x + 0.5), static_cast<unsigned int>(y + 0.5)));

    if (footprint_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(
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

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
