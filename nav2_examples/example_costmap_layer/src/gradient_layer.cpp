// Copyright 2024 Nav2 Contributors
// Licensed under the Apache License, Version 2.0

#include "example_costmap_layer/gradient_layer.hpp"

#include <cmath>
#include <algorithm>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace example_costmap_layer
{

GradientLayer::GradientLayer()
: gradient_center_x_(0.0),
  gradient_center_y_(0.0),
  gradient_radius_(5.0),
  gradient_strength_(1.0),
  enabled_(true),
  need_recalculation_(true)
{
}

void GradientLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in GradientLayer::onInitialize");
  }

  logger_ = node->get_logger();

  // Declare parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("gradient_center_x", rclcpp::ParameterValue(0.0));
  declareParameter("gradient_center_y", rclcpp::ParameterValue(0.0));
  declareParameter("gradient_radius", rclcpp::ParameterValue(5.0));
  declareParameter("gradient_strength", rclcpp::ParameterValue(1.0));

  // Get parameters
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".gradient_center_x", gradient_center_x_);
  node->get_parameter(name_ + ".gradient_center_y", gradient_center_y_);
  node->get_parameter(name_ + ".gradient_radius", gradient_radius_);
  node->get_parameter(name_ + ".gradient_strength", gradient_strength_);

  need_recalculation_ = true;
  current_ = true;

  RCLCPP_INFO(
    logger_,
    "GradientLayer initialized: center=(%.2f, %.2f), radius=%.2f, strength=%.2f",
    gradient_center_x_, gradient_center_y_, gradient_radius_, gradient_strength_);
}

void GradientLayer::activate()
{
  RCLCPP_INFO(logger_, "Activating GradientLayer");
}

void GradientLayer::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating GradientLayer");
}

void GradientLayer::reset()
{
  RCLCPP_INFO(logger_, "Resetting GradientLayer");
  need_recalculation_ = true;
  current_ = false;
}

bool GradientLayer::isClearable()
{
  return false;  // This layer doesn't support clearing
}

void GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  if (need_recalculation_) {
    // Expand bounds to include the entire gradient area
    *min_x = std::min(*min_x, gradient_center_x_ - gradient_radius_);
    *min_y = std::min(*min_y, gradient_center_y_ - gradient_radius_);
    *max_x = std::max(*max_x, gradient_center_x_ + gradient_radius_);
    *max_y = std::max(*max_y, gradient_center_y_ + gradient_radius_);
  }
}

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // Get costmap properties
  double resolution = master_grid.getResolution();
  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();

  // Iterate through the update region
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      // Convert cell indices to world coordinates
      double wx = origin_x + (i + 0.5) * resolution;
      double wy = origin_y + (j + 0.5) * resolution;

      // Calculate distance from gradient center
      double dx = wx - gradient_center_x_;
      double dy = wy - gradient_center_y_;
      double distance = std::hypot(dx, dy);

      // Skip cells outside the gradient radius
      if (distance > gradient_radius_) {
        continue;
      }

      // Calculate cost for this cell
      unsigned char cost = calculateCost(distance);

      // Get current cost in master grid
      unsigned char current_cost = master_grid.getCost(i, j);

      // Combine costs (use maximum)
      if (cost > current_cost && current_cost != nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(i, j, cost);
      } else if (current_cost == nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(i, j, cost);
      }
    }
  }

  need_recalculation_ = false;
  current_ = true;
}

void GradientLayer::matchSize()
{
  // Mark for recalculation when costmap size changes
  need_recalculation_ = true;
}

void GradientLayer::onFootprintChanged()
{
  // This layer doesn't depend on the footprint
  RCLCPP_DEBUG(logger_, "Footprint changed, no action needed for GradientLayer");
}

unsigned char GradientLayer::calculateCost(double distance)
{
  if (distance >= gradient_radius_) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  // Linear gradient from center to edge
  // Cost decreases as distance from center increases
  double normalized_dist = distance / gradient_radius_;
  double cost_factor = (1.0 - normalized_dist) * gradient_strength_;

  // Scale to costmap cost values (0-252, avoiding special values)
  unsigned char cost = static_cast<unsigned char>(
    std::min(252.0, cost_factor * 252.0));

  return cost;
}

}  // namespace example_costmap_layer

// Register the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example_costmap_layer::GradientLayer, nav2_costmap_2d::Layer)
