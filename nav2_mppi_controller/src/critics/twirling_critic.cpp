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

#include "nav2_mppi_controller/critics/twirling_critic.hpp"

#include <Eigen/Dense>

namespace mppi::critics
{

void TwirlingCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(enforce_path_inversion_, "enforce_path_inversion", false);

  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);

  RCLCPP_INFO(
    logger_, "TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

void TwirlingCritic::score(CriticData & data)
{
  if (!enabled_)
  {
    return;
  }

  geometry_msgs::msg::Pose active_goal;
  if (enforce_path_inversion_)
  {
    active_goal = utils::getLastPathPose(data.path);
  } else {
    active_goal = data.goal;
  }

  if (utils::withinPositionGoalTolerance(
      data.goal_checker, data.state.pose.pose, active_goal))
  {
    return;
  }

  if (power_ > 1u) {
    data.costs += ((data.state.wz.abs().rowwise().mean()) * weight_).pow(power_).eval();
  } else {
    data.costs += ((data.state.wz.abs().rowwise().mean()) * weight_).eval();
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TwirlingCritic,
  mppi::critics::CriticFunction)
