// Copyright (c) 2026 Open Navigation LLC
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

#include "nav2_regulated_pure_pursuit_controller/latency_compensator.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

// Commands older than the largest delay by this much are no longer needed
static constexpr double kHistoryMargin = 1.0;
static constexpr size_t kMaxHistorySize = 2000;
// Timestep of the forward simulation of the robot
static constexpr double kIntegrationDt = 0.01;

double AxisModel::steadyState(double command) const
{
  if (std::fabs(command) < deadband) {
    return 0.0;
  }
  return gain * command;
}

bool AxisModel::isIdentity() const
{
  return delay <= 0.0 && time_constant <= 0.0 && gain == 1.0 && deadband <= 0.0;
}

LatencyCompensator::LatencyCompensator(const AxisModel & linear, const AxisModel & angular)
: linear_(linear), angular_(angular)
{
}

void LatencyCompensator::reset()
{
  history_.clear();
  linear_deadband_residual_ = 0.0;
}

void LatencyCompensator::recordCommand(
  const geometry_msgs::msg::Twist & cmd, const rclcpp::Time & stamp)
{
  const double t = stamp.seconds();
  if (!history_.empty() && t < history_.back().stamp) {
    // Time jumped backwards (e.g. bag or simulation restart), so history is meaningless
    history_.clear();
  }

  if (!history_.empty() && t == history_.back().stamp) {
    history_.back() = {t, cmd.linear.x, cmd.angular.z};
  } else {
    history_.push_back({t, cmd.linear.x, cmd.angular.z});
  }

  // Keep the newest command older than the max delay, as it is still acting on the robot
  const double oldest_needed = t - std::max(linear_.delay, angular_.delay) - kHistoryMargin;
  while (history_.size() >= 2 &&
    (history_[1].stamp <= oldest_needed || history_.size() > kMaxHistorySize))
  {
    history_.pop_front();
  }
}

bool LatencyCompensator::commandAt(double t, double delay, StampedCommand & cmd) const
{
  // Input acting at t was issued at or before t - delay (zero order hold)
  const double issued_by = t - delay + 1e-9;
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (it->stamp <= issued_by) {
      cmd = *it;
      return true;
    }
  }
  return false;
}

PredictedState LatencyCompensator::predict(
  const geometry_msgs::msg::Twist & measured_speed, const rclcpp::Time & measurement_time,
  const rclcpp::Time & now) const
{
  PredictedState state;
  state.linear_vel = measured_speed.linear.x;
  state.angular_vel = measured_speed.angular.z;

  // Commands issued since the measurement act on the robot too, so simulate from the measurement
  const double age = std::max(0.0, (now - measurement_time).seconds());
  const double horizon = age + std::max(linear_.delay, angular_.delay);
  if (horizon <= 0.0) {
    return state;
  }

  const unsigned int steps = static_cast<unsigned int>(std::ceil(horizon / kIntegrationDt));
  const double dt = horizon / steps;
  const double alpha_lin =
    linear_.time_constant > 0.0 ? 1.0 - std::exp(-dt / linear_.time_constant) : 1.0;
  const double alpha_ang =
    angular_.time_constant > 0.0 ? 1.0 - std::exp(-dt / angular_.time_constant) : 1.0;

  const double t0 = measurement_time.seconds();
  StampedCommand cmd;
  for (unsigned int i = 0; i < steps; ++i) {
    const double t = t0 + i * dt;

    // Without a known in-flight command, assume the robot holds its current velocity.
    // When axis delays differ, the shorter one holds the most recent command past its delay.
    double target_lin = state.linear_vel;
    double target_ang = state.angular_vel;
    if (commandAt(t, linear_.delay, cmd)) {
      target_lin = linear_.steadyState(cmd.linear);
    }
    if (commandAt(t, angular_.delay, cmd)) {
      target_ang = angular_.steadyState(cmd.angular);
    }

    const double v = state.linear_vel + alpha_lin * (target_lin - state.linear_vel);
    const double w = state.angular_vel + alpha_ang * (target_ang - state.angular_vel);
    const double v_avg = 0.5 * (v + state.linear_vel);
    const double w_avg = 0.5 * (w + state.angular_vel);

    const double yaw_mid = state.yaw + 0.5 * w_avg * dt;
    state.x += v_avg * std::cos(yaw_mid) * dt;
    state.y += v_avg * std::sin(yaw_mid) * dt;
    state.yaw += w_avg * dt;
    state.linear_vel = v;
    state.angular_vel = w;
  }

  return state;
}

double LatencyCompensator::shapeCommand(
  double desired, double predicted, const AxisModel & model, double horizon,
  bool use_lead, bool invert_gain, bool allow_sign_change)
{
  if (desired == 0.0) {
    return 0.0;
  }

  // Constant command u over horizon h from v0 through a first-order lag gives
  // v(h) = u + (v0 - u) * e^(-h / tau), so solve v(h) = desired for u.
  double target = desired;
  if (use_lead && model.time_constant > 0.0 && horizon > 0.0) {
    const double decay = std::exp(-horizon / model.time_constant);
    target = (desired - predicted * decay) / (1.0 - decay);
  }

  // Unless allowed, never command against the desired direction; zero is the strongest brake
  if (!allow_sign_change && target * desired <= 0.0) {
    return 0.0;
  }

  if (invert_gain && model.gain > 0.0) {
    return target / model.gain;
  }
  return target;
}

double LatencyCompensator::roundToDeadband(double command, double deadband)
{
  if (std::fabs(command) >= deadband) {
    return command;
  }
  return std::fabs(command) >= 0.5 * deadband ? std::copysign(deadband, command) : 0.0;
}

double LatencyCompensator::modulateDeadband(double command, double deadband)
{
  if (command == 0.0 || std::fabs(command) >= deadband) {
    linear_deadband_residual_ = 0.0;
    return command;
  }

  // Round the command plus the error carried from previous rounding, then carry the new error
  const double target = command + linear_deadband_residual_;
  double output = std::fabs(target) >= 0.5 * deadband ? std::copysign(deadband, target) : 0.0;
  if (output * command < 0.0) {
    output = 0.0;
  }
  linear_deadband_residual_ = std::clamp(target - output, -deadband, deadband);
  return output;
}

double LatencyCompensator::rotateStoppingSpeed(
  double angle_remaining, double accel, double time_constant, double current_speed)
{
  // Without deceleration available the robot cannot stop, so it must not rotate
  if (accel <= 0.0) {
    return 0.0;
  }

  // Rotation still to come from the lag, positive when turning toward the target
  double lag_angle = 0.0;
  if (time_constant > 0.0) {
    lag_angle = time_constant * std::copysign(1.0, angle_remaining) * current_speed;
  }

  return std::sqrt(2.0 * accel * std::max(0.0, std::fabs(angle_remaining) - lag_angle));
}

}  // namespace nav2_regulated_pure_pursuit_controller
