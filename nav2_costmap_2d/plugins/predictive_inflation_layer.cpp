/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, User
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include "nav2_costmap_2d/predictive_inflation_layer.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::PredictiveInflationLayer, nav2_costmap_2d::Layer)

using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

PredictiveInflationLayer::PredictiveInflationLayer()
: InflationLayer(),
  predictive_mode_(false),
  nominal_speed_(0.5),
  speed_scale_(1.0),
  max_inflation_scale_(2.0),
  forward_angle_(1.0),
  side_angle_(2.3),
  forward_weight_(1.0),
  side_weight_(0.5),
  rear_weight_(0.2),
  base_inflation_radius_(0.0),
  last_robot_x_(0.0),
  last_robot_y_(0.0),
  last_robot_yaw_(0.0)
{
}

void PredictiveInflationLayer::onInitialize()
{
  // Initialize parent first (sets up node_, logger_, etc.)
  InflationLayer::onInitialize();

  loadParameters();

  // Store the "true" base radius configured by user
  base_inflation_radius_ = inflation_radius_;

  // STRATEGY: Max-Radius Expansion
  // We trick the parent class into calculating a larger inflation field
  // by setting its radius member to the maximum possible predictive reach.
  // This ensures the BFS queue propagates far enough.
  if (predictive_mode_) {
    inflation_radius_ = base_inflation_radius_ * max_inflation_scale_;
    matchSize();  // Recompute caches with new larger radius
  }

  // Register dynamic parameter callback
  // We remove the parent's handler and replace it with ours, which will call the parent's.
  auto node = node_.lock();
  if (node && dyn_params_handler_) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }

  if (node) {
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &PredictiveInflationLayer::dynamicParametersCallback,
        this, std::placeholders::_1));
  }
}

void PredictiveInflationLayer::loadParameters()
{
  declareParameter("predictive_mode", rclcpp::ParameterValue(false));
  declareParameter("nominal_speed", rclcpp::ParameterValue(0.5));
  declareParameter("speed_scale", rclcpp::ParameterValue(1.0));
  declareParameter("max_inflation_scale", rclcpp::ParameterValue(2.0));
  declareParameter("forward_angle", rclcpp::ParameterValue(1.0));
  declareParameter("side_angle", rclcpp::ParameterValue(2.3));
  declareParameter("forward_weight", rclcpp::ParameterValue(1.0));
  declareParameter("side_weight", rclcpp::ParameterValue(0.5));
  declareParameter("rear_weight", rclcpp::ParameterValue(0.2));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + ".predictive_mode", predictive_mode_);
  node->get_parameter(name_ + ".nominal_speed", nominal_speed_);
  node->get_parameter(name_ + ".speed_scale", speed_scale_);
  node->get_parameter(name_ + ".max_inflation_scale", max_inflation_scale_);
  node->get_parameter(name_ + ".forward_angle", forward_angle_);
  node->get_parameter(name_ + ".side_angle", side_angle_);
  node->get_parameter(name_ + ".forward_weight", forward_weight_);
  node->get_parameter(name_ + ".side_weight", side_weight_);
  node->get_parameter(name_ + ".rear_weight", rear_weight_);

  // Sanity clamps
  max_inflation_scale_ = std::max(1.0, max_inflation_scale_);
  forward_angle_ = std::clamp(forward_angle_, 0.0, M_PI);
  side_angle_ = std::clamp(side_angle_, forward_angle_, M_PI);
  forward_weight_ = std::max(0.0, forward_weight_);
  side_weight_ = std::max(0.0, side_weight_);
  rear_weight_ = std::max(0.0, rear_weight_);
}

rcl_interfaces::msg::SetParametersResult
PredictiveInflationLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // Lock mutex (parent uses the same mutex pointer if we didn't change it,
  // but parent uses getMutex(). We should use getMutex() too).
  // Note: InflationLayer::dynamicParametersCallback also locks the mutex.
  // std::recursive_mutex allows recursive locking.
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".predictive_mode") {
        predictive_mode_ = parameter.as_bool();
        need_reinflation_ = true;
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".nominal_speed") {
        nominal_speed_ = parameter.as_double();
      } else if (name == name_ + ".speed_scale") {
        speed_scale_ = parameter.as_double();
      } else if (name == name_ + ".max_inflation_scale") {
        max_inflation_scale_ = std::max(1.0, parameter.as_double());
        need_reinflation_ = true;
      } else if (name == name_ + ".forward_angle") {
        forward_angle_ = std::clamp(parameter.as_double(), 0.0, M_PI);
      } else if (name == name_ + ".side_angle") {
        side_angle_ = std::clamp(parameter.as_double(), forward_angle_, M_PI);
      } else if (name == name_ + ".forward_weight") {
        forward_weight_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".side_weight") {
        side_weight_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".rear_weight") {
        rear_weight_ = std::max(0.0, parameter.as_double());
      }
    }
  }

  // Call parent to handle standard parameters
  return InflationLayer::dynamicParametersCallback(parameters);
}

void PredictiveInflationLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // 1. Cache pose for updateCosts phase
  last_robot_x_ = robot_x;
  last_robot_y_ = robot_y;
  last_robot_yaw_ = robot_yaw;

  // 2. Standard Update
  // Since we expanded inflation_radius_ in onInitialize (or updateCosts),
  // the parent class will automatically expand the dirty window by the
  // larger radius. We don't need manual expansion here.
  InflationLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void PredictiveInflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // CRITICAL: Acquire lock (recursive with parent's lock)
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // 1. Parameter Consistency Check
  // Handle switching modes and dynamic parameter updates
  if (predictive_mode_) {
    double target_radius = base_inflation_radius_ * max_inflation_scale_;
    if (std::abs(inflation_radius_ - target_radius) > 1e-4) {
      // Radius differs from target.
      // Assumption: Current inflation_radius_ is the new user-desired base.
      base_inflation_radius_ = inflation_radius_;
      inflation_radius_ = base_inflation_radius_ * max_inflation_scale_;
      matchSize();  // Recompute caches
    }
  } else {
    // Predictive mode disabled. Ensure we are using the base radius.
    if (std::abs(inflation_radius_ - base_inflation_radius_) > 1e-4) {
      // Radius differs from base.
      double expanded_radius = base_inflation_radius_ * max_inflation_scale_;
      if (std::abs(inflation_radius_ - expanded_radius) < 1e-4) {
        // We are stuck in the expanded state from previous predictive mode.
        // Restore to base.
        inflation_radius_ = base_inflation_radius_;
        matchSize();
      } else {
        // User must have changed the parameter while mode=false.
        // Update our known base to match.
        base_inflation_radius_ = inflation_radius_;
      }
    }
  }

  // 2. Parent Inflation (Isotropic, Max Radius)
  InflationLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

  if (!predictive_mode_) {
    return;
  }

  // 3. Predictive Modulation
  unsigned char * master_array = master_grid.getCharMap();

  // Iterate over the bounds passed by the costmap (which should cover the update area)
  // We need to verify these bounds are safe
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const int index = master_grid.getIndex(i, j);
      const unsigned char base_cost = master_array[index];

      // Skip lethal/unknown/empty
      if (base_cost == NO_INFORMATION || base_cost == LETHAL_OBSTACLE || base_cost == 0) {
        continue;
      }

      // Geometry
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      const double dx = wx - last_robot_x_;
      const double dy = wy - last_robot_y_;

      // Transform to robot frame
      const double dx_r = std::cos(last_robot_yaw_) * dx + std::sin(last_robot_yaw_) * dy;
      const double dy_r = -std::sin(last_robot_yaw_) * dx + std::cos(last_robot_yaw_) * dy;
      const double angle = std::fabs(std::atan2(dy_r, dx_r));

      // Weighting
      double weight = computeDirectionalWeight(angle) * computeSpeedWeight();
      weight = std::clamp(weight, 0.01, max_inflation_scale_);

      // Re-calculate cost
      // We use effective distance scaling
      // NOTE: This logic handles the "Rear Clearing" requirement.
      // If the cell is in the rear (weight < 1.0) and was only visited
      // because of the global Max-Radius expansion, 'dist_effective'
      // will be large enough that 'computeCost' returns 0.
      const double dist_actual = costToDistance(base_cost);
      const double dist_effective = computeEffectiveDistance(dist_actual, weight);
      const unsigned char new_cost = computeCost(dist_effective);

      if (new_cost != base_cost) {
        master_array[index] = new_cost;
      }
    }
  }
}

double PredictiveInflationLayer::computeDirectionalWeight(double angle) const
{
  if (angle <= forward_angle_) {
    return forward_weight_;
  } else if (angle <= side_angle_) {
    return side_weight_;
  }
  return rear_weight_;
}

double PredictiveInflationLayer::computeSpeedWeight() const
{
  const double weight = 1.0 + (nominal_speed_ * speed_scale_);
  return std::max(1.0, weight);
}

double PredictiveInflationLayer::computeEffectiveDistance(
  double actual_distance,
  double weight) const
{
  const double safe_weight = std::max(weight, 0.01);
  return actual_distance / safe_weight;
}

double PredictiveInflationLayer::costToDistance(unsigned char cost) const
{
  if (cost == LETHAL_OBSTACLE) {return 0.0;}
  if (cost == NO_INFORMATION) {return std::numeric_limits<double>::infinity();}
  if (cost >= INSCRIBED_INFLATED_OBSTACLE) {return inscribed_radius_ / resolution_;}
  if (cost == 0) {return std::numeric_limits<double>::infinity();}

  // cost = (INSCRIBED_INFLATED_OBSTACLE - 1) * factor
  // factor = exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_))

  double factor = static_cast<double>(cost) / static_cast<double>(INSCRIBED_INFLATED_OBSTACLE - 1);

  // ln(factor) = -scaling * (dist_meters - inscribed)
  // (dist_meters - inscribed) = ln(factor) / -scaling
  // dist_meters = inscribed - (ln(factor) / scaling)

  double dist_meters = inscribed_radius_ - (std::log(factor) / cost_scaling_factor_);

  return dist_meters / resolution_;
}

}  // namespace nav2_costmap_2d
