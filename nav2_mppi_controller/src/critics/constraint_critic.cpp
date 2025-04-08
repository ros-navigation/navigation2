// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2025 Open Navigation LLC
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

#include "nav2_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0f);
  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5f);
  getParentParam(vy_max, "vy_max", 0.0f);
  getParentParam(vx_min, "vx_min", -0.35f);
  getParentParam(ax_min_, "ax_min", 0.0f);
  getParentParam(ax_max_, "ax_max", 0.0f);
  getParentParam(ay_min_, "ay_min", 0.0f);
  getParentParam(ay_max_, "ay_max", 0.0f);

  if (abs(ax_min_) < 1e-3f || abs(ax_max_) < 1e-3f) {
    use_acc_ = false;
    RCLCPP_INFO(
      logger_, "ConstraintCritic will not use acceleration constraints.");
  } else {
    use_acc_ = true;
    RCLCPP_INFO(
      logger_, "ConstraintCritic will use acceleration constraints.");
  }

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

void ConstraintCritic::applyAccelerationConstraints(
  const float min_accel,
  const float max_accel,
  const float init_value,
  const Eigen::ArrayXXf & velocities,
  const float model_dt,
  Eigen::ArrayXf & costs)
{
  float max_delta = model_dt * max_accel;
  float min_delta = model_dt * min_accel;
  Eigen::ArrayXf last_vx = Eigen::ArrayXf::Constant(
    velocities.rows(),
    init_value);

  unsigned int n_cols = velocities.cols();
  for (unsigned int i = 1; i < n_cols; i++) {
    const auto & curr_vx = velocities.col(i);
    const auto & dv = curr_vx - last_vx;

    auto high_accel_penalty = (curr_vx >= 0.0f).select(
      (dv - max_delta).max(0.0f),
      (-max_delta - dv).max(0.0f));
    auto low_accel_penalty = (curr_vx >= 0.0f).select(
      (min_delta - dv).max(0.0f),
      (dv + min_delta).max(0.0f));

    if (power_ > 1u) {
      costs += (((high_accel_penalty + low_accel_penalty) * model_dt)
        .rowwise().sum().eval() * weight_).pow(power_).eval();
    } else {
      costs +=
        (((high_accel_penalty + low_accel_penalty) * model_dt)
        .rowwise().sum().eval() * weight_).eval();
    }
    last_vx = curr_vx;
  }
}

void ConstraintCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // Differential motion model
  auto diff = dynamic_cast<DiffDriveMotionModel *>(data.motion_model.get());
  if (diff != nullptr) {
    if (power_ > 1u) {
      data.costs += (((((data.state.vx - max_vel_).max(0.0f) + (min_vel_ - data.state.vx).
        max(0.0f)) * data.model_dt).rowwise().sum().eval()) * weight_).pow(power_).eval();
    } else {
      data.costs += (((((data.state.vx - max_vel_).max(0.0f) + (min_vel_ - data.state.vx).
        max(0.0f)) * data.model_dt).rowwise().sum().eval()) * weight_).eval();
    }

    if (use_acc_) {
      applyAccelerationConstraints(ax_min_, ax_max_,
        data.state.speed.linear.x, data.state.vx, data.model_dt, data.costs);
    }

    return;
  }

  // Omnidirectional motion model
  auto omni = dynamic_cast<OmniMotionModel *>(data.motion_model.get());
  if (omni != nullptr) {
    auto & vx = data.state.vx;
    unsigned int n_rows = data.state.vx.rows();
    unsigned int n_cols = data.state.vx.cols();
    Eigen::ArrayXXf sgn(n_rows, n_cols);
    sgn = vx.unaryExpr([](const float x){return copysignf(1.0f, x);});

    auto vel_total = sgn * (data.state.vx.square() + data.state.vy.square()).sqrt();
    if (power_ > 1u) {
      data.costs += ((((vel_total - max_vel_).max(0.0f) + (min_vel_ - vel_total).
        max(0.0f)) * data.model_dt).rowwise().sum().eval() * weight_).pow(power_).eval();
    } else {
      data.costs += ((((vel_total - max_vel_).max(0.0f) + (min_vel_ - vel_total).
        max(0.0f)) * data.model_dt).rowwise().sum().eval() * weight_).eval();
    }

    if (use_acc_) {
      applyAccelerationConstraints(ax_min_, ax_max_,
        data.state.speed.linear.x, data.state.vx, data.model_dt, data.costs);
      applyAccelerationConstraints(ay_min_, ay_max_,
        data.state.speed.linear.y, data.state.vy, data.model_dt, data.costs);
    }

    return;
  }

  // Ackermann motion model
  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    float min_turning_rad = acker->getMinTurningRadius();
    auto out_of_turning_rad_motion = (min_turning_rad - (vx.abs() / wz.abs())).max(0.0f);
    if (power_ > 1u) {
      data.costs += ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
        out_of_turning_rad_motion) * data.model_dt).rowwise().sum().eval() *
        weight_).pow(power_).eval();
    } else {
      data.costs += ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
        out_of_turning_rad_motion) * data.model_dt).rowwise().sum().eval() * weight_).eval();
    }

    if (use_acc_) {
      applyAccelerationConstraints(ax_min_, ax_max_,
        data.state.speed.linear.x, data.state.vx, data.model_dt, data.costs);
    }

    return;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
