// Copyright (c) 2026 Rem3000
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
//
// AI disclosure: this file was drafted with AI assistance (Claude) and reviewed by the author.

#include "nav2_mppi_controller/critics/axis_align_critic.hpp"

namespace mppi::critics
{

void AxisAlignCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 3.0f);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  getParam(normalize_, "normalize", true);

  RCLCPP_INFO(
    logger_,
    "AxisAlignCritic instantiated with %d power and %f weight, %s scaling.",
    power_, weight_, normalize_ ? "ratio" : "absolute");
}

void AxisAlignCritic::score(CriticData & data)
{
  if (!enabled_ || !data.motion_model->isHolonomic() ||
    data.state.local_path_length < threshold_to_consider_)
  {
    return;
  }

  const auto vx = data.state.vx.abs();
  const auto vy = data.state.vy.abs();

  Eigen::ArrayXf diagonal;
  if (normalize_) {
    // Ratio of the minor to the major body-axis velocity: 0 for axis-aligned motion,
    // exactly 1 at 45 degrees. Independent of speed, so the critic does not also
    // discourage driving fast (the absolute form below does, which is rarely wanted).
    // The major axis is clamped to a small floor only to avoid dividing by zero at rest.
    diagonal = (vx.min(vy) / vx.max(vy).max(1e-3f)).rowwise().mean();
  } else {
    diagonal = vx.min(vy).rowwise().mean();
  }

  if (power_ > 1u) {
    data.costs += (diagonal * weight_).pow(power_);
  } else {
    data.costs += diagonal * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::AxisAlignCritic, mppi::critics::CriticFunction)
