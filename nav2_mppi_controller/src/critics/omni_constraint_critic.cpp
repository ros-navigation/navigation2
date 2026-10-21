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

#include "nav2_mppi_controller/critics/omni_constraint_critic.hpp"

namespace mppi::critics
{

void OmniConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0f);
  RCLCPP_INFO(
    logger_, "OmniConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  getParentParam(vx_max_, "vx_max", 0.5f);
  getParentParam(vy_max_, "vy_max", 0.0f);
  getParentParam(vx_min_, "vx_min", -0.35f);
}

void OmniConstraintCritic::score(CriticData & data)
{
  if (!enabled_ || !data.motion_model->isHolonomic()) {
    return;
  }

  auto & vx = data.state.vx;
  auto & vy = data.state.vy;

  // (vx/vx_max)² + (vy/vy_max)², which is 1 exactly on the ellipse.
  const auto normalized_sq =
    (vx.max(0.0f) * utils::safeInverseAbs(vx_max_) +
    vx.min(0.0f) * utils::safeInverseAbs(vx_min_)).square() +
    (vy * utils::safeInverseAbs(vy_max_)).square();

  // The speed given up in scaling the sample back onto the ellipse
  const auto violation =
    (vx.square() + vy.square()).sqrt() * (1.0f - normalized_sq.max(1.0f).rsqrt());

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
  mppi::critics::OmniConstraintCritic,
  mppi::critics::CriticFunction)
