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
#include <iterator>

#include "nav2_regulated_pure_pursuit_controller/latency_compensator.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

LatencyCompensator::LatencyCompensator(Parameters * params)
: params_(params)
{
}

bool LatencyCompensator::isEnabled() const
{
  return params_->model_delay_vx > 0.0 || params_->model_delay_wz > 0.0 ||
         params_->model_time_constant_vx > 0.0 || params_->model_time_constant_wz > 0.0;
}

void LatencyCompensator::reset()
{
  history_.clear();
}

void LatencyCompensator::recordCommand(
  const geometry_msgs::msg::Twist & cmd, const rclcpp::Time & stamp)
{
  const double t = stamp.seconds();
  const double last_t =
    history_.empty() ? 0.0 : rclcpp::Time(history_.back().header.stamp).seconds();
  if (!history_.empty() && t < last_t) {
    // Time jumped backwards (e.g. bag or simulation restart), so history is meaningless
    history_.clear();
  }

  geometry_msgs::msg::TwistStamped stamped_cmd;
  stamped_cmd.header.stamp = stamp;
  stamped_cmd.twist = cmd;
  if (!history_.empty() && t == last_t) {
    history_.back() = stamped_cmd;
  } else {
    history_.push_back(stamped_cmd);
  }

  // Keep the newest command older than the max delay, as it is still acting on the robot
  const double oldest_needed =
    t - std::max(params_->model_delay_vx, params_->model_delay_wz) - 1.0;
  while (history_.size() >= 2 &&
    rclcpp::Time(history_[1].header.stamp).seconds() <= oldest_needed)
  {
    history_.pop_front();
  }
}

const geometry_msgs::msg::Twist * LatencyCompensator::commandAt(double t, double delay) const
{
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (rclcpp::Time(it->header.stamp).seconds() <= t - delay + 1e-9) {
      return &it->twist;
    }
  }
  return nullptr;
}

PredictedState LatencyCompensator::predict(
  const geometry_msgs::msg::Twist & measured_speed, const rclcpp::Time & measurement_time,
  const rclcpp::Time & now) const
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  double v = measured_speed.linear.x;
  double w = measured_speed.angular.z;

  // Commands issued since the measurement act on the robot too, so simulate from the measurement.
  // Path tracking acts through the turning rate, so predict to when an angular command issued now
  // takes effect, rather than a longer linear delay, which would track from too far ahead.
  const double horizon =
    std::max(0.0, (now - measurement_time).seconds()) + params_->model_delay_wz;
  if (horizon > 0.0) {
    constexpr double integration_dt = 0.01;
    const unsigned int steps = static_cast<unsigned int>(std::ceil(horizon / integration_dt));
    const double dt = horizon / steps;
    const double tau_v = params_->model_time_constant_vx;
    const double tau_w = params_->model_time_constant_wz;
    const double alpha_v = tau_v > 0.0 ? 1.0 - std::exp(-dt / tau_v) : 1.0;
    const double alpha_w = tau_w > 0.0 ? 1.0 - std::exp(-dt / tau_w) : 1.0;
    const double t0 = measurement_time.seconds();

    for (unsigned int i = 0; i < steps; ++i) {
      const double t = t0 + i * dt;

      // Without a known in-flight command, assume the robot holds its current velocity.
      // When axis delays differ, the shorter one holds the most recent command past its delay.
      double target_v = v;
      double target_w = w;
      if (const auto * cmd = commandAt(t, params_->model_delay_vx)) {
        target_v = cmd->linear.x;
      }
      if (const auto * cmd = commandAt(t, params_->model_delay_wz)) {
        target_w = cmd->angular.z;
      }

      const double v_next = v + alpha_v * (target_v - v);
      const double w_next = w + alpha_w * (target_w - w);
      const double v_avg = 0.5 * (v + v_next);
      const double w_avg = 0.5 * (w + w_next);
      const double yaw_mid = yaw + 0.5 * w_avg * dt;
      x += v_avg * std::cos(yaw_mid) * dt;
      y += v_avg * std::sin(yaw_mid) * dt;
      yaw += w_avg * dt;
      v = v_next;
      w = w_next;
    }
  }

  PredictedState state;
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  state.velocity.linear.x = v;
  state.velocity.angular.z = w;
  return state;
}

nav_msgs::msg::Path transformPlanToPredictedFrame(
  const nav_msgs::msg::Path & path, const PredictedState & prediction)
{
  tf2::Transform base_to_predicted;
  tf2::fromMsg(prediction.pose, base_to_predicted);
  geometry_msgs::msg::TransformStamped transform;
  transform.header = path.header;
  transform.transform = tf2::toMsg(base_to_predicted.inverse());

  nav_msgs::msg::Path predicted_path;
  predicted_path.header = path.header;
  predicted_path.poses.resize(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    tf2::doTransform(path.poses[i], predicted_path.poses[i], transform);
  }

  // Drop the path behind the predicted pose, as the controller server does for the robot pose,
  // since the lookahead point is found by integrating distance from the first pose.
  // Search only as far as the robot could have moved so we don't skip ahead to a later portion
  // of the path, and keep at least 2 points to retain the end-of-path direction.
  if (predicted_path.poses.size() > 2) {
    const double search_dist =
      2.0 * std::hypot(prediction.pose.position.x, prediction.pose.position.y) + 0.5;
    auto search_end = nav2_util::geometry_utils::first_after_integrated_distance(
      predicted_path.poses.begin(), predicted_path.poses.end(), search_dist);
    auto closest = nav2_util::geometry_utils::min_by(
      predicted_path.poses.begin(), search_end,
      [](const geometry_msgs::msg::PoseStamped & ps) {
        return std::hypot(ps.pose.position.x, ps.pose.position.y);
      });
    if (closest == std::prev(predicted_path.poses.end())) {
      closest = std::prev(closest);
    }
    predicted_path.poses.erase(predicted_path.poses.begin(), closest);
  }

  return predicted_path;
}

}  // namespace nav2_regulated_pure_pursuit_controller
