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
  getParam(weight_, "cost_weight", 35.0);

  // Recast double to float
  std::vector<double> deadband_velocities{0.0, 0.0, 0.0};
  getParam(deadband_velocities, "deadband_velocities", std::vector<double>{0.0, 0.0, 0.0});
  std::transform(
    deadband_velocities.begin(), deadband_velocities.end(), deadband_velocities_.begin(),
    [](double d) {return static_cast<float>(d);});

  RCLCPP_INFO_STREAM(
    logger_, "VelocityDeadbandCritic instantiated with "
      << power_ << " power, " << weight_ << " weight, deadband_velocity ["
      << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
      << deadband_velocities_.at(2) << "]");
}

void VelocityDeadbandCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  if (data.motion_model->isHolonomic()) {
    if (power_ > 1u) {
      data.costs += ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
        (fabs(deadband_velocities_[1]) - data.state.vy.abs()).max(0.0f) +
        (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
        data.model_dt).rowwise().sum() * weight_).pow(power_);
    } else {
      data.costs += (((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
        (fabs(deadband_velocities_[1]) - data.state.vy.abs()).max(0.0f) +
        (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
        data.model_dt).rowwise().sum() * weight_;
    }
    return;
  }

  if (power_ > 1u) {
    data.costs += ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
      (fabs(deadband_velocities_[1]) - data.state.vy.abs()).max(0.0f) +
      (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
      data.model_dt).rowwise().sum() * weight_).pow(power_);
  } else {
    data.costs += (((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
      (fabs(deadband_velocities_[1]) - data.state.vy.abs()).max(0.0f) +
      (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
      data.model_dt).rowwise().sum() * weight_;
  }
  return;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::VelocityDeadbandCritic, mppi::critics::CriticFunction)
