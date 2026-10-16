// Copyright (c) 2026 Karinca Robotics
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

#ifndef OPENNAV_DOCKING__CONTROLLER_PLUGINS__PID_CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_PLUGINS__PID_CONTROLLER_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opennav_docking/controller_base.hpp"

namespace opennav_docking
{

/**
 * @class opennav_docking::PIDController
 * @brief A docking controller implementing PID controller.
 *
 * Calculation of the heading error in poseError method is based on
 * "On uniform semiglobal exponential stability of proportional line-of-sight
 * guidance laws" (Fossen and Pettersen, Automatica, 2014).
 *
 */
class PIDController : public ControllerBase
{
public:
  /**
   * @brief Zero internal state.
   *
   * Called by the server on entry to each control loop and once per docking retry
   */
  void reset() override;

  /**
   * @brief Activate the parameter callbacks.
   */
  void activate() override;

  /**
   * @brief Deactivate the parameter callbacks.
   */
  void deactivate() override;

  /**
   * @brief Declare and read the parameters.
   * @param node Lifecycle node
   */
  void onConfigure(const nav2::LifecycleNode::SharedPtr & node) override;

  /**
   * @brief Apply the PID law to produce a velocity command.
   *
   * @param target Target pose, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @param dt Control loop sampling period.
   * @returns Command velocity.
   */
  geometry_msgs::msg::Twist computeCommand(
    const geometry_msgs::msg::Pose & target, bool reverse, double dt) override;

  /**
   * @brief Simulate one step of the PID law for collision checking.
   *
   * @param dt Simulation time step [s].
   * @param target Target pose, in robot centric coordinates.
   * @param current Current pose along the projection, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @returns The pose after one step forward.
   */
  geometry_msgs::msg::Pose predictNextPose(
    double dt, const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current, bool reverse) override;

  /**
   * @struct ControllerState
   * @brief One PID axis: its gains, its anti-windup bound and its running state.
   */
  struct ControllerState
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double i_clamp{0.0};

    double integral{0.0};
    double prev_error{0.0};
    double derivative{0.0};

    void reset()
    {
      integral = 0.0;
      prev_error = 0.0;
      derivative = 0.0;
    }
  };

  /**
   * @brief Evaluate one axis without touching its state.
   *
   * @param channel The controller state to evaluate.
   * @param error Current error on this axis.
   * @param integral Integrator value to use.
   * @param derivative Derivative value to use.
   * @returns The axis's contribution to the command.
   */
  static double evaluate(
    const ControllerState & channel, double error, double integral, double derivative);

  /**
   * @brief Advance one axis's integrator and derivative by one step.
   *
   * @param channel The controller state to advance.
   * @param error Current error on this axis.
   * @param dt Time step [s]
   */
  double advance(ControllerState & channel, double error, double dt);

protected:
  /// Hard floor under lookahead_distance_.
  static constexpr double kMinLookahead = 1e-3;

  /**
   * @brief Calculate the distance and the heading error to the target.
   *
   * @param target Target pose, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @param side Sign of the lateral term, becomes -1 when it passes target.
   * @param e_x Distance to the target along the robot's heading [m]
   * @param alpha Heading error relative to the target's x axis [rad].
   */
  void poseError(
    const geometry_msgs::msg::Pose & target, bool reverse, double & side,
    double & e_x, double & alpha) const;

  /**
   * @brief Validate parameter updates before applying.
   * @param parameters List of parameters.
   * @return The update is accepted or not.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation.
   * @param parameters List of parameters.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;

  ControllerState x_, y_;
  double v_linear_max_{0.25}, v_angular_max_{0.75};
  double lookahead_distance_{0.25};
  // Distance e_x must pass zero before the control direction switches [m]
  double passed_target_hysteresis_{0.05};
  // Sign of the lateral term used in poseError
  double side_{1.0};
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_PLUGINS__PID_CONTROLLER_HPP_
