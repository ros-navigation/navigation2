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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATOR_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATOR_HPP_

#include <deque>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @struct PredictedState
 * @brief Robot state when a command issued now takes effect
 */
struct PredictedState
{
  // Relative to the measured robot pose
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist velocity;
};

/**
 * @class nav2_regulated_pure_pursuit_controller::LatencyCompensator
 * @brief Predicts the robot state when a command issued now takes effect (Smith predictor),
 * by replaying the commands in flight through a delay and first-order lag on each axis
 */
class LatencyCompensator
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::LatencyCompensator
   * @param params Parameters holding the model of the robot's response
   */
  explicit LatencyCompensator(Parameters * params);

  /**
   * @brief Whether a model of the robot's response is set, enabling compensation
   */
  bool isEnabled() const;

  /**
   * @brief Clear the commands issued
   */
  void reset();

  /**
   * @brief Record a command issued, and drop those no longer acting on the robot
   * @param cmd Command issued
   * @param stamp Time the command was issued
   */
  void recordCommand(const geometry_msgs::msg::Twist & cmd, const rclcpp::Time & stamp);

  /**
   * @brief Predict the robot state when a command issued now takes effect
   * @param measured_speed Measured robot velocity
   * @param measurement_time Time of the robot pose and velocity measurement
   * @param now Current time
   * @return Predicted state relative to the measured robot pose
   */
  PredictedState predict(
    const geometry_msgs::msg::Twist & measured_speed, const rclcpp::Time & measurement_time,
    const rclcpp::Time & now) const;

protected:
  struct StampedCommand
  {
    double stamp;
    double linear;
    double angular;
  };

  /**
   * @brief Get the command acting on an axis at a time, issued at or before the time less the
   * axis delay
   */
  const StampedCommand * commandAt(double t, double delay) const;

  Parameters * params_;
  std::deque<StampedCommand> history_;
};

/**
 * @brief Re-express a robot-frame path in the frame of the predicted robot pose and drop
 * the poses behind its closest point
 * @param path Path in the robot base frame
 * @param prediction Predicted robot state relative to the robot base frame
 * @return Path in the predicted robot frame
 */
nav_msgs::msg::Path transformPlanToPredictedFrame(
  const nav_msgs::msg::Path & path, const PredictedState & prediction);

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATOR_HPP_
