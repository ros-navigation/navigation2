// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{


void GoalCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0f);
  getParam(threshold_to_consider_, "threshold_to_consider", 1.4f);

  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void GoalCritic::score(CriticData & data)
{
  if (!enabled_ || data.state.local_path_length > threshold_to_consider_) {
    return;
  }

  geometry_msgs::msg::Pose goal = utils::getLastPathPose(data.path);

  // Use euclidean distance for cost calculation instead of integral path (it's faster)
  // When within threshold_to_consider_ of the goal, euclidean distance provides
  // a computationally efficient approximation with negligible accuracy loss.
  auto goal_x = goal.position.x;
  auto goal_y = goal.position.y;

  const auto delta_x = data.trajectories.x - goal_x;
  const auto delta_y = data.trajectories.y - goal_y;

  if(power_ > 1u) {
    data.costs += (((delta_x.square() + delta_y.square()).sqrt()).rowwise().mean() *
      weight_).pow(power_);
  } else {
    data.costs += (((delta_x.square() + delta_y.square()).sqrt()).rowwise().mean() *
      weight_).eval();
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
