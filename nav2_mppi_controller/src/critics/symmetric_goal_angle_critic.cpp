// Copyright (c) 2025
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

#include "nav2_mppi_controller/critics/symmetric_goal_angle_critic.hpp"
#include "angles/angles.h"

namespace mppi::critics
{

void SymmetricGoalAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5);

  RCLCPP_INFO(
    logger_,
    "SymmetricGoalAngleCritic instantiated with %d power, %f weight, and %f "
    "angular threshold.",
    power_, weight_, threshold_to_consider_);
}

void SymmetricGoalAngleCritic::score(CriticData & data)
{
  if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);
  const float goal_yaw_flipped = angles::normalize_angle(goal_yaw + M_PI);

  // Calculate angular distance to both the goal orientation and flipped orientation
  auto distance_to_goal = xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw));
  auto distance_to_flipped = xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw_flipped));

  // Use the minimum distance (allowing the robot to approach from either direction)
  auto min_distance = xt::minimum(distance_to_goal, distance_to_flipped);

  // Apply the cost function
  data.costs += xt::pow(xt::mean(min_distance, {1}) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::SymmetricGoalAngleCritic,
  mppi::critics::CriticFunction)
