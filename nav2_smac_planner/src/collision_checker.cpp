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

#include <unordered_set>
#include <cmath>
#include <algorithm>

#include "nav2_util/line_iterator.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  nav2::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
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
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
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
    return true;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(
    static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED_COST || center_cost_ == OCCUPIED_COST) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    float footprint_cost = static_cast<float>(footprintCost(current_footprint));

    if (footprint_cost == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost >= OCCUPIED_COST;
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED_COST;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED_COST;
}

bool GridCollisionChecker::inCollision(
  const float & x0, const float & y0, const float & theta0,
  const float & x1, const float & y1, const float & theta1,
  const bool & traverse_unknown,
  const float & min_turning_radius)
{
  // Convert angle bins to radians
  float start_theta = angles_[static_cast<unsigned int>(theta0)];
  float end_theta = angles_[static_cast<unsigned int>(theta1)];
  float dtheta = end_theta - start_theta;
  if (dtheta > M_PI) {
    dtheta -= 2.0f * static_cast<float>(M_PI);
  } else if (dtheta < -M_PI) {
    dtheta += 2.0f * static_cast<float>(M_PI);
  }

  // Quick checks to see if sweeping is necessary. If both poses are well
  // clear of obstacles, fall back to the single pose check on the child node
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();

  if (outsideRange(size_x, x0) || outsideRange(size_y, y0) ||
    outsideRange(size_x, x1) || outsideRange(size_y, y1))
  {
    return true;
  }

  unsigned int mx0 = static_cast<unsigned int>(x0 + 0.5f);
  unsigned int my0 = static_cast<unsigned int>(y0 + 0.5f);
  unsigned int mx1 = static_cast<unsigned int>(x1 + 0.5f);
  unsigned int my1 = static_cast<unsigned int>(y1 + 0.5f);
  float start_cost = static_cast<float>(costmap_->getCost(mx0, my0));
  float end_cost = static_cast<float>(costmap_->getCost(mx1, my1));

  bool parent_safe = possible_collision_cost_ > 0.0f &&
    start_cost < possible_collision_cost_;
  bool child_safe = possible_collision_cost_ > 0.0f &&
    end_cost < possible_collision_cost_;

  if (parent_safe && child_safe) {
    return inCollision(x1, y1, theta1, traverse_unknown);
  }

  float dx = x1 - x0;
  float dy = y1 - y0;
  float distance = hypotf(dx, dy);
  float arc = fabsf(dtheta) * min_turning_radius;
  int steps = static_cast<int>(ceilf(std::max(std::max(distance, arc), 1.0f)));

  std::unordered_set<unsigned int> cells;
  float cx = 0.0f, cy = 0.0f;
  const bool use_arc = fabsf(dtheta) > 1e-3f;
  if (use_arc) {
    if (dtheta > 0.0f) {
      cx = x0 - min_turning_radius * sin(start_theta);
      cy = y0 + min_turning_radius * cos(start_theta);
    } else {
      cx = x0 + min_turning_radius * sin(start_theta);
      cy = y0 - min_turning_radius * cos(start_theta);
    }
  }

  for (int step = 0; step <= steps; ++step) {
    float t = static_cast<float>(step) / static_cast<float>(steps);
    float xi, yi, theta;
    if (use_arc) {
      theta = start_theta + t * dtheta;
      if (dtheta > 0.0f) {
        xi = cx + min_turning_radius * sin(theta);
        yi = cy - min_turning_radius * cos(theta);
      } else {
        xi = cx - min_turning_radius * sin(theta);
        yi = cy + min_turning_radius * cos(theta);
      }
    } else {
      xi = x0 + t * dx;
      yi = y0 + t * dy;
      theta = start_theta;
    }

    while (theta < 0.0f) {
      theta += 2.0f * static_cast<float>(M_PI);
    }
    while (theta >= 2.0f * static_cast<float>(M_PI)) {
      theta -= 2.0f * static_cast<float>(M_PI);
    }
    unsigned int angle_bin = static_cast<unsigned int>(theta / (2.0f * static_cast<float>(M_PI)) * angles_.size());
    angle_bin %= angles_.size();

    if (footprint_is_radius_) {
      if (outsideRange(costmap_->getSizeInCellsX(), xi) ||
        outsideRange(costmap_->getSizeInCellsY(), yi))
      {
        return true;
      }
      unsigned int mx = static_cast<unsigned int>(xi + 0.5f);
      unsigned int my = static_cast<unsigned int>(yi + 0.5f);
      cells.insert(my * size_x + mx);
      continue;
    }

    // Always include center cell
    if (outsideRange(costmap_->getSizeInCellsX(), xi) ||
      outsideRange(costmap_->getSizeInCellsY(), yi))
    {
      return true;
    }
    unsigned int mx = static_cast<unsigned int>(xi + 0.5f);
    unsigned int my = static_cast<unsigned int>(yi + 0.5f);
    cells.insert(my * size_x + mx);

    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(xi), static_cast<double>(yi), wx, wy);
    const nav2_costmap_2d::Footprint & oriented = oriented_footprints_[angle_bin];

    unsigned int x_prev, y_prev, x_curr, y_curr;
    if (!worldToMap(wx + oriented[0].x, wy + oriented[0].y, x_prev, y_prev)) {
      return true;
    }
    for (unsigned int i = 1; i < oriented.size(); ++i) {
      if (!worldToMap(wx + oriented[i].x, wy + oriented[i].y, x_curr, y_curr)) {
        return true;
      }
      for (nav2_util::LineIterator line(x_prev, y_prev, x_curr, y_curr); line.isValid(); line.advance()) {
        cells.insert(line.getY() * size_x + line.getX());
      }
      x_prev = x_curr;
      y_prev = y_curr;
    }
    if (!worldToMap(wx + oriented[0].x, wy + oriented[0].y, x_curr, y_curr)) {
      return true;
    }
    for (nav2_util::LineIterator line(x_prev, y_prev, x_curr, y_curr); line.isValid(); line.advance()) {
      cells.insert(line.getY() * size_x + line.getX());
    }
  }

  float max_cost = 0.0f;
  for (auto index : cells) {
    center_cost_ = static_cast<float>(costmap_->getCost(index));
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      continue;
    }
    if (center_cost_ >= INSCRIBED_COST) {
      return true;
    }
    if (center_cost_ > max_cost) {
      max_cost = center_cost_;
    }
  }

  center_cost_ = max_cost;
  return false;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
