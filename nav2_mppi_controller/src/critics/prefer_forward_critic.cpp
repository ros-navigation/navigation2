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

#include "nav2_mppi_controller/critics/prefer_forward_critic.hpp"

#include <Eigen/Dense>

namespace mppi::critics
{

void PreferForwardCritic::initialize()
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
    logger_, "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  float distance = utils::getCriticGoalPathDistance(data, enforce_path_inversion_);
  
  if (distance > threshold_to_consider_) {
    return;
  }

  if (power_ > 1u) {
    data.costs += (
      (data.state.vx.unaryExpr([&](const float & x){return std::max(-x, 0.0f);}) *
      data.model_dt).rowwise().sum() * weight_).pow(power_);
  } else {
    data.costs += (data.state.vx.unaryExpr([&](const float & x){return std::max(-x, 0.0f);}) *
      data.model_dt).rowwise().sum() * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
