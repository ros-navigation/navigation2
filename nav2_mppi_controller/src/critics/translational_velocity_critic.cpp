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

#include "nav2_mppi_controller/critics/translational_velocity_critic.hpp"

namespace mppi::critics
{

namespace
{

/**
 * @brief Inverse square of a velocity limit, floored so that a zero limit cannot divide by zero
 * @param limit Velocity limit of either sign
 * @return 1 / limit², at most 1e12
 */
float invSquare(const float limit)
{
  constexpr float min_limit = 1e-6f;
  return 1.0f / std::max(limit * limit, min_limit * min_limit);
}

}  // namespace

void TranslationalVelocityCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0f);
  RCLCPP_INFO(
    logger_, "TranslationalVelocityCritic instantiated with %d power and %f weight.",
    power_, weight_);

  getParentParam(vx_max_, "vx_max", 0.5f);
  getParentParam(vy_max_, "vy_max", 0.0f);
  getParentParam(vx_min_, "vx_min", -0.35f);
}

void TranslationalVelocityCritic::score(CriticData & data)
{
  if (!enabled_ || !data.motion_model->isHolonomic()) {
    return;
  }

  auto & vx = data.state.vx;
  auto & vy = data.state.vy;

  // (vx/vx_max)² + (vy/vy_max)², which is 1 exactly on the ellipse. The forward and reverse
  // semi-axes differ, so vx is split by direction of travel.
  const auto normalized_sq =
    invSquare(vx_max_) * vx.max(0.0f).square() +
    invSquare(vx_min_) * vx.min(0.0f).square() +
    invSquare(vy_max_) * vy.square();

  // |v| - |v| / sqrt(normalized_sq), the excess speed along the direction of travel, clamped so
  // that velocities inside the ellipse cost nothing
  const auto violation =
    (vx.square() + vy.square()).sqrt() * (1.0f - normalized_sq.max(1.0f).rsqrt());

  const Eigen::ArrayXf cost = violation.rowwise().sum() * data.model_dt * weight_;
  if (power_ > 1u) {
    data.costs += cost.pow(power_);
  } else {
    data.costs += cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TranslationalVelocityCritic,
  mppi::critics::CriticFunction)
