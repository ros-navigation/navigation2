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

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @struct AxisModel
 * @brief Command-response model of a single velocity axis, in Hammerstein form:
 * command -> deadband -> gain -> pure delay -> first-order lag -> velocity.
 *
 * The deadband is a cut-off: commands with magnitude below it produce no motion and
 * commands above it pass through unshifted. The delay lumps transport, actuation and
 * odometry latency into a single deadtime.
 */
struct AxisModel
{
  double delay{0.0};
  double time_constant{0.0};
  double gain{1.0};
  double deadband{0.0};

  /**
   * @brief Steady-state velocity the axis converges to under a constant command
   */
  double steadyState(double command) const;

  /**
   * @brief Whether the model is the identity (instant, exact tracking)
   */
  bool isIdentity() const;
};

/**
 * @struct PredictedState
 * @brief Robot state at the time the next command takes effect, relative to the current
 * robot base frame
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
 * @class nav2_regulated_pure_pursuit_controller::LatencyCompensator
 * @brief Smith-predictor style latency compensation for velocity-commanded robots.
 *
 * Keeps a time-stamped history of issued commands and forward-simulates a unicycle through the
 * per-axis AxisModel to predict where the robot will be, and how fast it will be moving, when a
 * command issued now begins to take effect. It can also shape a desired velocity into a command
 * by inverting the lag (lead), gain and deadband of the model.
 *
 * It depends on no controller-specific types so that it may be shared with other controllers.
 */
class LatencyCompensator
{
public:
  /**
   * @brief Constructor
   * @param linear Linear (x) axis model, which may be updated while in use
   * @param angular Angular (z) axis model, which may be updated while in use
   */
  LatencyCompensator(const AxisModel & linear, const AxisModel & angular);

  /**
   * @brief Clear the command history
   */
  void reset();

  /**
   * @brief Record a command sent to the robot
   * @param cmd Command velocity
   * @param stamp Time the command was issued
   */
  void recordCommand(const geometry_msgs::msg::Twist & cmd, const rclcpp::Time & stamp);

  /**
   * @brief Predict the robot state when a command issued at `now` takes effect
   * @param measured_speed Current measured robot velocity
   * @param now Current time, on the same clock as the recorded commands
   * @return State relative to the current robot base frame
   */
  PredictedState predict(
    const geometry_msgs::msg::Twist & measured_speed, const rclcpp::Time & now) const;

  /**
   * @brief Shape a desired velocity into the command that best achieves it under the model
   * @param desired Desired axis velocity
   * @param predicted Predicted axis velocity when the command takes effect
   * @param model Axis model
   * @param horizon Time over which the lead term attempts to reach the desired velocity
   * @param use_lead Whether to invert the first-order lag
   * @param invert_actuation Whether to invert the gain and deadband
   * @return Command for the axis. Never of opposite sign to the desired velocity.
   */
  static double shapeCommand(
    double desired, double predicted, const AxisModel & model, double horizon,
    bool use_lead, bool invert_actuation);

  /**
   * @brief Largest commanded rotation speed from which the robot can stop within an angle.
   *
   * Ramping the command down from w at deceleration a through a first-order lag with time
   * constant tau, starting from a robot velocity w0, rotates the robot by
   * w^2 / (2a) + tau * w0, since the integral of a first-order lag's output exceeds that of
   * its input by tau * (initial - final output). Solves that for w.
   * @param angle_remaining Angle left to rotate (rad)
   * @param accel Angular deceleration limit (rad/s^2)
   * @param time_constant Time constant of the angular lag (s)
   * @param current_speed Robot angular velocity when the command takes effect (rad/s)
   * @return Maximum angular velocity magnitude (rad/s)
   */
  static double rotateStoppingSpeed(
    double angle_remaining, double accel, double time_constant = 0.0,
    double current_speed = 0.0);

protected:
  struct StampedCommand
  {
    double stamp;
    double linear;
    double angular;
  };

  /**
   * @brief Find the command input acting on the plant at time t for a given axis delay
   * @return Whether such a command exists in the history
   */
  bool commandAt(double t, double delay, StampedCommand & cmd) const;

  const AxisModel & linear_;
  const AxisModel & angular_;
  std::deque<StampedCommand> history_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__LATENCY_COMPENSATOR_HPP_
