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

#include "nav2_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{

void VelocityDeadbandCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  getParam(deadband_velocities_, "deadband_velocities", std::vector<double>{ 0.0, 0.0, 0.0 });
  RCLCPP_INFO_STREAM(logger_, "VelocityDeadbandCritic instantiated with "
                                << power_ << " power, " << weight_ << " weight, deadband_velocity [" << deadband_velocities_.at(0)
                                << "," << deadband_velocities_.at(1) << "," << deadband_velocities_.at(2) << "]");
}

void VelocityDeadbandCritic::score(CriticData& data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_)
  {
    return;
  }

  auto& vx = data.state.vx;
  auto& wz = data.state.wz;

  auto out_of_deadband_vx_motion = xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0);
  auto out_of_deadband_wz_motion = xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0);

  if (data.motion_model->isHolonomic()) {
    auto& vy = data.state.vy;
    auto out_of_deadband_vy_motion = xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0);
    data.costs += xt::pow(
      xt::sum((std::move(out_of_deadband_vx_motion) + std::move(out_of_deadband_vy_motion) + 
      std::move(out_of_deadband_wz_motion)) * data.model_dt, { 1 }, immediate) *
      weight_, power_);
    return;
  }

  data.costs += xt::pow(
    xt::sum((std::move(out_of_deadband_vx_motion) + std::move(out_of_deadband_wz_motion)) * data.model_dt, { 1 }, immediate) *
    weight_, power_);
  return;
}

} // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::VelocityDeadbandCritic, mppi::critics::CriticFunction)
