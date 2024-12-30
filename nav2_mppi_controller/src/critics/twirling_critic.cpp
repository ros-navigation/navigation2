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

namespace mppi::critics
{

void TwirlingCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);

  RCLCPP_INFO(
    logger_, "TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

void TwirlingCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose.pose, data.goal))
  {
    return;
  }

  if (power_ > 1u) {
    data.costs += xt::pow(xt::mean(xt::fabs(data.state.wz), {1}, immediate) * weight_, power_);
  } else {
    data.costs += xt::mean(xt::fabs(data.state.wz), {1}, immediate) * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TwirlingCritic,
  mppi::critics::CriticFunction)
