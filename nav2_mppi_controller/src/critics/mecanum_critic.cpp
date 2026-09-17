// Copyright (c) 2026 Duatic AG
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

#include <algorithm>
#include <cmath>

#include "nav2_mppi_controller/critics/mecanum_critic.hpp"

namespace mppi::critics
{

namespace
{

/**
 * @brief Reciprocal of a velocity limit, floored so that a zero limit cannot divide by zero
 * @param limit Velocity limit of either sign
 * @return 1 / |limit|, at most 1e6
 */
float invLimit(const float limit)
{
  constexpr float min_limit = 1e-6f;
  return 1.0f / std::max(std::abs(limit), min_limit);
}

}  // namespace

void MecanumCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0f);
  RCLCPP_INFO(
    logger_, "MecanumCritic instantiated with %d power and %f weight.",
    power_, weight_);

  getParam(vx_max_, "vx_max", 0.5f);
  getParam(vy_max_, "vy_max", 0.5f);
  getParam(wz_max_, "wz_max", 1.7f);
  getParam(center_projection_, "sum_of_robot_center_projection_on_X_Y_axis", 0.3f);
}

void MecanumCritic::score(CriticData & data)
{
  if (!enabled_ || !data.motion_model->isHolonomic()) {
    return;
  }

  auto & vx = data.state.vx;
  auto & vy = data.state.vy;
  auto & wz = data.state.wz;

  // |vx|/vx_max + |vy|/vy_max + |wz|/wz_max, which is 1 exactly on the wheel limit.
  const auto normalized =
    vx.abs() * invLimit(vx_max_) +
    vy.abs() * invLimit(vy_max_) + wz.abs() * invLimit(wz_max_);

  // The speed given up in scaling the sample back onto the wheel limit
  const auto violation =
    ((vx.square() + vy.square()).sqrt() + wz.abs() * center_projection_) *
    (1.0f - normalized.max(1.0f).inverse());

  // Use column loop to preserve vectorization.
  Eigen::ArrayXf cost = Eigen::ArrayXf::Zero(vx.rows());
  for (int i = 0; i < vx.cols(); ++i) {
    cost += violation.col(i);
  }
  cost *= data.model_dt * weight_;
  if (power_ > 1u) {
    data.costs += cost.pow(power_);
  } else {
    data.costs += cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::MecanumCritic,
  mppi::critics::CriticFunction)
