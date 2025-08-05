// Copyright (c) 2025, Angsa Robotics GmbH
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

#include "nav2_mppi_controller/collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include <algorithm>
#include <cmath>

namespace nav2_mppi_controller
{

MPPICollisionChecker::MPPICollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }
}

void MPPICollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const std::string & inflation_layer_name)
{
  // Calculate the circumscribed cost automatically
  possible_collision_cost_ = findCircumscribedCost(inflation_layer_name);

  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // Store the unoriented footprint for on-the-fly transformation
  unoriented_footprint_ = footprint;
}

CollisionResult MPPICollisionChecker::inCollision(
  const std::vector<float> & x,
  const std::vector<float> & y,
  const std::vector<float> & yaw,
  const bool & traverse_unknown)
{
  CollisionResult result;
  result.in_collision = false;

  // Check if all vectors have the same size
  if (x.size() != y.size() || x.size() != yaw.size()) {
    result.in_collision = true;
    return result;
  }

  // Initialize result vectors
  result.center_cost.resize(x.size(), -1.0f);
  result.footprint_cost.resize(x.size(), -1.0f);

  // Step 1: Check all poses for bounds and get center costs
  std::vector<bool> needs_footprint_check(x.size(), false);

  for (size_t i = 0; i < x.size(); ++i) {
    // Check to make sure cell is inside the map
    if (outsideRange(costmap_->getSizeInCellsX(), x[i]) ||
      outsideRange(costmap_->getSizeInCellsY(), y[i]))
    {
      result.in_collision = true;
      return result;
    }

    // Get center cost for this pose
    float current_center_cost = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x[i] + 0.5f), static_cast<unsigned int>(y[i] + 0.5f)));

    result.center_cost[i] = current_center_cost;

    if (current_center_cost == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown) {
      result.in_collision = true;
      return result;
    }

    if (current_center_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      result.in_collision = true;
      return result;
    }

    // For footprint-based collision checking, mark poses that need further checking
    if (!footprint_is_radius_) {
      // Skip if center cost is below collision threshold
      if (current_center_cost >= possible_collision_cost_ || possible_collision_cost_ <= 0.0f) {
        needs_footprint_check[i] = true;
      }
    }
  }

  if (footprint_is_radius_) {
    return result;  // No further checks needed for radius-based checking
  }

  // Step 2: If using footprint, check footprint costs for poses that need it
  std::vector<bool> needs_swept_area_check(x.size(), false);
  // Cache computed footprints to avoid recomputation in Step 4
  std::vector<nav2_costmap_2d::Footprint> cached_footprints(x.size());

  for (size_t i = 0; i < x.size(); ++i) {
    // Skip poses that don't need footprint checking
    if (!needs_footprint_check[i]) {
      continue;
    }

    // Transform footprint to current orientation and world position
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x[i]), static_cast<double>(y[i]), wx, wy);

    // Transform the footprint to the given orientation
    nav2_costmap_2d::Footprint oriented_footprint = transformFootprint(unoriented_footprint_,
        yaw[i]);

    // Translate to world position
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    geometry_msgs::msg::Point new_pt;
    for (unsigned int j = 0; j < oriented_footprint.size(); ++j) {
      new_pt.x = wx + oriented_footprint[j].x;
      new_pt.y = wy + oriented_footprint[j].y;
      current_footprint.push_back(new_pt);
    }

    // Cache the computed footprint for potential reuse in Step 4
    cached_footprints[i] = current_footprint;

    // Check footprint perimeter
    float footprint_cost = static_cast<float>(footprintCost(current_footprint, false));

    // Store footprint cost in result
    result.footprint_cost[i] = footprint_cost;

    if (footprint_cost == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown) {
      result.in_collision = true;
      return result;
    }

    if (footprint_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      result.in_collision = true;
      return result;
    }

    // Mark for swept area checking if footprint cost is INSCRIBED_INFLATED_OBSTACLE
    if (footprint_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      needs_swept_area_check[i] = true;
    }
  }

  // Step 3: Check swept area for consecutive poses with footprint cost INSCRIBED_INFLATED_OBSTACLE
  // Find consecutive sequences of poses that need swept area checking
  std::vector<std::vector<size_t>> consecutive_sequences;
  std::vector<size_t> current_sequence;

  for (size_t i = 0; i < x.size(); ++i) {
    if (needs_swept_area_check[i]) {
      current_sequence.push_back(i);
      // Limit sequence to maximum of 5 poses to avoid an unprecise swept area check
      if (current_sequence.size() >= 5) {
        consecutive_sequences.push_back(current_sequence);
        current_sequence.clear();
      }
    } else {
      if (!current_sequence.empty()) {
        // Only add sequences with more than one pose
        if (current_sequence.size() > 1) {
          consecutive_sequences.push_back(current_sequence);
        }
        current_sequence.clear();
      }
    }
  }

  // Don't forget the last sequence if it ends at the last pose
  if (!current_sequence.empty()) {
    // Only add sequences with more than one pose
    if (current_sequence.size() > 1) {
      consecutive_sequences.push_back(current_sequence);
    }
  }

  // Step 4: Check swept area using convex hull for each consecutive sequence
  for (const auto & sequence : consecutive_sequences) {
    // Collect all footprint points from consecutive poses using cached footprints
    std::vector<geometry_msgs::msg::Point> all_points;

    for (size_t seq_idx : sequence) {
      // Use cached footprint instead of recomputing
      const nav2_costmap_2d::Footprint & current_footprint = cached_footprints[seq_idx];

      for (const auto & footprint_pt : current_footprint) {
        all_points.push_back(footprint_pt);
      }
    }

    // Create convex hull from all collected points
    nav2_costmap_2d::Footprint convex_hull = createConvexHull(all_points);

    // Check swept area cost using full area checking
    float swept_area_cost = static_cast<float>(footprintCost(convex_hull, true));

    if (swept_area_cost == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown) {
      result.in_collision = true;
      return result;
    }

    if (swept_area_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      result.in_collision = true;
      return result;
    }
  }

  return result;
}

nav2_costmap_2d::Footprint MPPICollisionChecker::createConvexHull(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  if (points.size() <= 1) {
    return points;
  }

  // Create a copy of points for sorting
  std::vector<geometry_msgs::msg::Point> sorted_points = points;

  // Sort points lexicographically (first by x, then by y)
  std::sort(sorted_points.begin(), sorted_points.end(),
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });

  // Remove duplicate points
  sorted_points.erase(
    std::unique(sorted_points.begin(), sorted_points.end(),
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
      return std::abs(a.x - b.x) < 1e-9 && std::abs(a.y - b.y) < 1e-9;
      }),
    sorted_points.end());

  if (sorted_points.size() <= 1) {
    return sorted_points;
  }

  // Andrew's monotone chain algorithm
  std::vector<geometry_msgs::msg::Point> hull;

  // Helper function to compute cross product
  auto cross = [](const geometry_msgs::msg::Point & O,
    const geometry_msgs::msg::Point & A,
    const geometry_msgs::msg::Point & B) {
      return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    };

  // Build lower hull
  for (size_t i = 0; i < sorted_points.size(); ++i) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull[hull.size() - 1],
        sorted_points[i]) <= 0)
    {
      hull.pop_back();
    }
    hull.push_back(sorted_points[i]);
  }

  // Build upper hull
  size_t t = hull.size() + 1;
  for (int i = static_cast<int>(sorted_points.size()) - 2; i >= 0; --i) {
    while (hull.size() >= t && cross(hull[hull.size() - 2], hull[hull.size() - 1],
        sorted_points[i]) <= 0)
    {
      hull.pop_back();
    }
    hull.push_back(sorted_points[i]);
  }

  // Remove last point because it's the same as the first one
  if (hull.size() > 1) {
    hull.pop_back();
  }

  return hull;
}

bool MPPICollisionChecker::outsideRange(const unsigned int & max, const float & value) const
{
  return value < 0.0f || value > max;
}

nav2_costmap_2d::Footprint MPPICollisionChecker::transformFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  float yaw) const
{
  double sin_th = sin(yaw);
  double cos_th = cos(yaw);
  nav2_costmap_2d::Footprint oriented_footprint;
  oriented_footprint.reserve(footprint.size());

  geometry_msgs::msg::Point new_pt;
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    new_pt.x = footprint[i].x * cos_th - footprint[i].y * sin_th;
    new_pt.y = footprint[i].x * sin_th + footprint[i].y * cos_th;
    oriented_footprint.push_back(new_pt);
  }

  return oriented_footprint;
}

float MPPICollisionChecker::findCircumscribedCost(const std::string & inflation_layer_name)
{
  if (!costmap_ros_) {
    RCLCPP_ERROR(logger_, "Costmap ROS is not available for circumscribed cost calculation");
    return -1.0f;
  }

  double result = -1.0;
  const double circum_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();

  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  const auto inflation_layer = nav2_costmap_2d::InflationLayer::getInflationLayer(
    costmap_ros_,
    inflation_layer_name);

  if (inflation_layer != nullptr) {
    const double resolution = costmap_ros_->getCostmap()->getResolution();
    double inflation_radius = inflation_layer->getInflationRadius();

    if (inflation_radius < circum_radius) {
      RCLCPP_ERROR(
        logger_,
        "The inflation radius (%f) is smaller than the circumscribed radius (%f) "
        "If this is an SE2-collision checking plugin, it cannot use costmap potential "
        "field to speed up collision checking by only checking the full footprint "
        "when robot is within possibly-inscribed radius of an obstacle. This may "
        "significantly slow down planning times!",
        inflation_radius, circum_radius);
      result = 0.0;
    } else {
      result = inflation_layer->computeCost(circum_radius / resolution);
    }
  } else {
    RCLCPP_WARN(
      logger_,
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

}  // namespace nav2_mppi_controller
