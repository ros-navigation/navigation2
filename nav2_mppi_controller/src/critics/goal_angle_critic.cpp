// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"
#include "angles/angles.h"
namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0f);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(symmetric_yaw_tolerance_, "symmetric_yaw_tolerance", false);

  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, %f "
    "angular threshold and symmetric_yaw_tolerance %s",
    power_, weight_, threshold_to_consider_, symmetric_yaw_tolerance_ ? "enabled" : "disabled");
}

void GoalAngleCritic::score(CriticData & data)
{
  if (!enabled_ || data.state.local_path_length > threshold_to_consider_) {
    return;
  }

  geometry_msgs::msg::Pose goal = utils::getLastPathPose(data.path);

  double goal_yaw = tf2::getYaw(goal.orientation);

  auto angular_distances = utils::shortest_angular_distance(data.trajectories.yaws,
    goal_yaw).abs().eval();

  if (symmetric_yaw_tolerance_) {
    double symmetric_goal_yaw = angles::normalize_angle(goal_yaw + M_PI);
    auto symmetric_distances = utils::shortest_angular_distance(data.trajectories.yaws,
      symmetric_goal_yaw).abs().eval();
    angular_distances = angular_distances.min(symmetric_distances);
  }

  if (power_ > 1u) {
    data.costs += ((angular_distances.rowwise().mean()) * weight_).pow(power_).eval();
  } else {
    data.costs += ((angular_distances.rowwise().mean()) * weight_).eval();
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GoalAngleCritic,
  mppi::critics::CriticFunction)
