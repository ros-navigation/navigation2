// Copyright (c) 2024 Enway GmbH, Adi Vardi
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

#include "nav2_mppi_controller/critics/direction_change_critic.hpp"

#include <Eigen/Dense>

namespace mppi::critics
{

void DirectionChangeCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(enforce_path_inversion_, "enforce_path_inversion", false);

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0f);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5f);

  RCLCPP_INFO(
    logger_, "DirectionChangeCritic instantiated with %d power and %f weight.", power_, weight_);
}

void DirectionChangeCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  geometry_msgs::msg::Pose goal = utils::getCriticGoal(data, enforce_path_inversion_);

  if (utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, goal))
  {
    return;
  }

  // Penalize the magnitude of velocity difference when crossing zero (direction change)
  // Calculate |vx - current_speed| only where signs differ, otherwise 0

  constexpr size_t penalize_up_to_idx = 2;
  const float current_speed = data.state.speed.linear.x;
  // Process in-place using Eigen views to avoid allocations
  auto vx_view = data.state.vx.leftCols(penalize_up_to_idx);

  // TODO also penalize change direction in wz (and vy for holonomic case) . maybe add a flag to enable/disable wz
  if (power_ > 1u) {
    data.costs += ((vx_view * current_speed < 0.0f).select(
      (vx_view - current_speed).abs(), 0.0f).rowwise().sum() * weight_).pow(power_);
  } else {
    data.costs += (vx_view * current_speed < 0.0f).select(
      (vx_view - current_speed).abs(), 0.0f).rowwise().sum() * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::DirectionChangeCritic,
  mppi::critics::CriticFunction)
