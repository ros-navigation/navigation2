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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATION_FUNCTIONS_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATION_FUNCTIONS_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

namespace latency_compensation
{

struct StampedCommand
{
  double stamp;
  double linear;
  double angular;
};

using CommandHistory = std::deque<StampedCommand>;

/**
 * @struct PredictedState
 * @brief Robot state when a command issued now takes effect, relative to the measured pose
 */
struct PredictedState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_vel{0.0};
  double angular_vel{0.0};
};

/**
 * @brief Add a command to the history and drop commands no longer acting on the robot
 * @param history   Commands issued, oldest first
 * @param cmd       Command issued
 * @param stamp     Time (s) the command was issued
 * @param max_delay Largest delay (s) of the robot's response on any axis
 */
inline void recordCommand(
  CommandHistory & history, const geometry_msgs::msg::Twist & cmd,
  const double & stamp, const double & max_delay)
{
  if (!history.empty() && stamp < history.back().stamp) {
    // Time jumped backwards (e.g. bag or simulation restart), so history is meaningless
    history.clear();
  }

  if (!history.empty() && stamp == history.back().stamp) {
    history.back() = {stamp, cmd.linear.x, cmd.angular.z};
  } else {
    history.push_back({stamp, cmd.linear.x, cmd.angular.z});
  }

  // Keep the newest command older than the max delay, as it is still acting on the robot
  const double oldest_needed = stamp - max_delay - 1.0;
  while (history.size() >= 2 && history[1].stamp <= oldest_needed) {
    history.pop_front();
  }
}

/**
 * @brief Predict the robot state when a command issued now takes effect (Smith predictor),
 *        by replaying the commands in flight through a delay and first-order lag on each axis
 * @param history          Commands issued, oldest first
 * @param measured_speed   Measured robot velocity
 * @param measurement_time Time (s) of the robot pose and velocity measurement
 * @param now              Current time (s)
 * @param linear_delay     Delay (s) of the robot's linear velocity response
 * @param angular_delay    Delay (s) of the robot's angular velocity response
 * @param linear_tau       Time constant (s) of the robot's linear velocity response
 * @param angular_tau      Time constant (s) of the robot's angular velocity response
 * @return                 Predicted state relative to the measured pose
 */
inline PredictedState predictState(
  const CommandHistory & history, const geometry_msgs::msg::Twist & measured_speed,
  const double & measurement_time, const double & now,
  const double & linear_delay, const double & angular_delay,
  const double & linear_tau, const double & angular_tau)
{
  PredictedState state;
  state.linear_vel = measured_speed.linear.x;
  state.angular_vel = measured_speed.angular.z;

  // Commands issued since the measurement act on the robot too, so simulate from the measurement.
  // Path tracking acts through the turning rate, so predict to when an angular command issued now
  // takes effect, rather than a longer linear delay, which would track from too far ahead.
  const double horizon = std::max(0.0, now - measurement_time) + angular_delay;
  if (horizon <= 0.0) {
    return state;
  }

  // Command acting at time t, issued at or before t - delay (zero order hold)
  auto commandAt = [&](double t, double delay) -> const StampedCommand * {
      for (auto it = history.rbegin(); it != history.rend(); ++it) {
        if (it->stamp <= t - delay + 1e-9) {
          return &(*it);
        }
      }
      return nullptr;
    };

  constexpr double integration_dt = 0.01;
  const unsigned int steps = static_cast<unsigned int>(std::ceil(horizon / integration_dt));
  const double dt = horizon / steps;
  const double alpha_lin = linear_tau > 0.0 ? 1.0 - std::exp(-dt / linear_tau) : 1.0;
  const double alpha_ang = angular_tau > 0.0 ? 1.0 - std::exp(-dt / angular_tau) : 1.0;

  for (unsigned int i = 0; i < steps; ++i) {
    const double t = measurement_time + i * dt;

    // Without a known in-flight command, assume the robot holds its current velocity.
    // When axis delays differ, the shorter one holds the most recent command past its delay.
    double target_lin = state.linear_vel;
    double target_ang = state.angular_vel;
    if (const auto * cmd = commandAt(t, linear_delay)) {
      target_lin = cmd->linear;
    }
    if (const auto * cmd = commandAt(t, angular_delay)) {
      target_ang = cmd->angular;
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

/**
 * @brief Re-express a robot-frame path in the frame of the predicted robot pose and drop
 *        the poses behind its closest point
 * @param path       Path in the robot base frame
 * @param prediction Predicted robot state relative to the robot base frame
 * @return           Path in the predicted robot frame
 */
inline nav_msgs::msg::Path transformPlanToPredictedFrame(
  const nav_msgs::msg::Path & path, const PredictedState & prediction)
{
  nav_msgs::msg::Path predicted_path;
  predicted_path.header = path.header;
  predicted_path.poses.reserve(path.poses.size());

  const double c = std::cos(prediction.yaw);
  const double s = std::sin(prediction.yaw);
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped p = pose;
    const double dx = pose.pose.position.x - prediction.x;
    const double dy = pose.pose.position.y - prediction.y;
    p.pose.position.x = c * dx + s * dy;
    p.pose.position.y = -s * dx + c * dy;
    p.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      tf2::getYaw(pose.pose.orientation) - prediction.yaw);
    predicted_path.poses.push_back(p);
  }

  // Drop the path behind the predicted pose, as the controller server does for the robot pose,
  // since the lookahead point is found by integrating distance from the first pose.
  // Search only as far as the robot could have moved so we don't skip ahead to a later portion
  // of the path, and keep at least 2 points to retain the end-of-path direction.
  if (predicted_path.poses.size() > 2) {
    const double search_dist = 2.0 * std::hypot(prediction.x, prediction.y) + 0.5;
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

}  // namespace latency_compensation

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATION_FUNCTIONS_HPP_
