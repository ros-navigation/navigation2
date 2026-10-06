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
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opennav_docking/controller_base.hpp"

namespace opennav_docking
{

/**
 * @struct PIDGains
 * @brief PID controller parameters.
 */
struct PIDGains
{
  double kp;
  double ki;
  double kd;
  double i_clamp;
};

/**
 * @struct PIDParameters
 * @brief Parameters in the controller namespace.
 */
struct PIDParameters
{
  PIDGains x;
  PIDGains y;
  PIDGains theta;
  double v_linear_max;
  double v_angular_max;
  double lookahead_distance;
};

/**
 * @class opennav_docking::PIDParameterHandler
 * @brief Handles parameters of the PIDController.
 */
class PIDParameterHandler : public nav2_util::ParameterHandler<PIDParameters>
{
public:
  /**
   * @brief Declare the PID law parameters.
   * @param node Lifecycle node
   * @param name The controller parameter namespace
   * @param logger Logger
   */
  PIDParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node, const std::string & name,
    const rclcpp::Logger & logger);

  /**
   * @brief Check if parameters changed since the last call with mutex held.
   * @return True if the parameters were updated
   */
  bool isUpdated() {return std::exchange(updated_, false);}

protected:
  /**
   * @brief Validate parameters before applying.
   * @param parameters List of parameters.
   * @return rcl_interfaces::msg::SetParametersResult Result of the update request.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Update parameter after validation
   * @param parameters List of parameters to be updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  std::string name_;
  bool updated_{false};
};

/**
 * @class opennav_docking::PIDController
 * @brief A docking controller implementing PID controller.
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
   * @brief Cleanup the PID law parameters.
   */
  void cleanup() override;

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

    ControllerState & operator=(const PIDGains & gains)
    {
      kp = gains.kp;
      ki = gains.ki;
      kd = gains.kd;
      i_clamp = gains.i_clamp;
      return *this;
    }

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

  /**
   * @brief Calculate target pose in polar coordinate.
   *
   * @param target Target pose, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @param lookahead Floor on denominator [m] in bearing calculation.
   * @param rho Range to the target [m].
   * @param alpha Bearing to the target, in (-pi/2, pi/2) [rad].
   * @param alignment Line of sight relative to the target's axis [rad].
   */
  static void poseError(
    const geometry_msgs::msg::Pose & target, bool reverse, double lookahead,
    double & rho, double & alpha, double & alignment);

protected:
  /// Hard floor under lookahead_distance_.
  static constexpr double kMinLookahead = 1e-3;

  /**
   * @brief Copy the latest parameters to working values.
   * Call with the parameter handler's mutex held.
   */
  void applyParameters();

  ControllerState x_, y_, theta_;
  double v_linear_max_{0.25}, v_angular_max_{0.75};
  double lookahead_distance_{0.25};

  std::unique_ptr<PIDParameterHandler> pid_param_handler_;
  PIDParameters * pid_params_{nullptr};
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_PLUGINS__PID_CONTROLLER_HPP_
