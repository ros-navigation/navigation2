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

  precomputed_collision_cells_.clear();
  precomputed_collision_cells_.reserve(angles_.size());

  if (footprint_is_radius_) {
    // For circular footprint, all angles have the same single center cell
    std::vector<std::pair<int, int>> center_cell = {{0, 0}};
    for (unsigned int i = 0; i != angles_.size(); i++) {
      precomputed_collision_cells_.push_back(center_cell);
    }
    return;
  }

  // Resolution inverse
  const double inv_resolution = 1.0 / costmap_->getResolution();

  for (unsigned int i = 0; i != angles_.size(); i++) {
    robin_hood::unordered_set<std::pair<int, int>, PairHash> unique_cells;

    // Rotate footprint vertices for this angle
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint.size());
    nav2_costmap_2d::transformFootprint(0.0, 0.0, angles_[i], footprint, oriented_footprint);

    // Convert footprint vertices to map coordinates (assuming robot at origin)
    std::vector<std::pair<int, int>> footprint_cells;
    footprint_cells.reserve(oriented_footprint.size());

    for (const auto & vertex : oriented_footprint) {
      // Convert world coordinates to map cell coordinates
      const int mx = static_cast<int>(std::lround(vertex.x * inv_resolution));
      const int my = static_cast<int>(std::lround(vertex.y * inv_resolution));
      footprint_cells.emplace_back(mx, my);
    }

      // Rasterize all edges using LineIterator and collect unique cells
    for (unsigned int j = 0; j < footprint_cells.size(); j++) {
      const unsigned int next_j = (j + 1) % footprint_cells.size();
      const auto & start = footprint_cells[j];
      const auto & end = footprint_cells[next_j];

      // Use LineIterator to get all cells along this edge
      for (nav2_util::LineIterator line_it(start.first, start.second, end.first, end.second);
        line_it.isValid(); line_it.advance())
      {
        unique_cells.emplace(line_it.getX(), line_it.getY());
      }
    }

    // Convert robin_hood set to vector
    std::vector<std::pair<int, int>> cells_for_angle;
    cells_for_angle.reserve(unique_cells.size());
    cells_for_angle.assign(unique_cells.begin(), unique_cells.end());

    precomputed_collision_cells_.push_back(std::move(cells_for_angle));
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

  // Calculate robot cell coordinates
  const int robot_mx = static_cast<int>(x + 0.5f);
  const int robot_my = static_cast<int>(y + 0.5f);

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(robot_mx, robot_my));

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
    // Use precomputed collision cells
    const unsigned int angle_idx = static_cast<unsigned int>(angle_bin);
    const auto & cells_to_check = precomputed_collision_cells_[angle_idx];

    // Bounds
    const int max_x = static_cast<int>(costmap_->getSizeInCellsX());
    const int max_y = static_cast<int>(costmap_->getSizeInCellsY());
    unsigned char max_footprint_cost = 0;

    for (const auto & cell_offset : cells_to_check) {
      const int check_mx = robot_mx + cell_offset.first;
      const int check_my = robot_my + cell_offset.second;

      // Bounds check for each footprint cell
      if (check_mx < 0 || check_mx >= max_x || check_my < 0 || check_my >= max_y) {
        // Out of bounds cells are treated as lethal obstacles
        return true;
      }

      const unsigned char cell_cost = costmap_->getCost(check_mx, check_my);
      max_footprint_cost = std::max(max_footprint_cost, cell_cost);
    }

    const float footprint_cost = static_cast<float>(max_footprint_cost);

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
