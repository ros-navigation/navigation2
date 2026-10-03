// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#ifndef OPENNAV_DOCKING__CONTROLLER_PLUGINS__GRACEFUL_CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_PLUGINS__GRACEFUL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "opennav_docking/controller_base.hpp"

namespace opennav_docking
{

/**
 * @struct GracefulParameters
 * @brief Parameters of the smooth control law.
 */
struct GracefulParameters
{
  double k_phi;
  double k_delta;
  double beta;
  double lambda;
  double v_linear_min;
  double v_linear_max;
  double v_angular_max;
  double slowdown_radius;
  double deceleration_max;
};

/**
 * @class opennav_docking::GracefulParameterHandler
 * @brief Handles the parameters and dynamic parameters of GracefulController.
 */
class GracefulParameterHandler : public nav2_util::ParameterHandler<GracefulParameters>
{
public:
  /**
   * @brief Declare parameters in controller namespace.
   * @param node Lifecycle node
   * @param name The controller's parameter namespace
   * @param logger Logger
   */
  GracefulParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node, const std::string & name,
    const rclcpp::Logger & logger);

  /**
   * @brief Check if parameters changed since the last call with mutex held.
   * @return True if the parameters were updated
   */
  bool isUpdated() {return std::exchange(updated_, false);}

protected:
  /**
   * @brief Validate parameter before applying.
   * @param parameters List of parameters to update.
   * @return rcl_interfaces::msg::SetParametersResult Result of the update.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Apply parameter after validation
   * @param parameters List of parameters to be updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  std::string name_;
  bool updated_{false};
};

/**
 * @class opennav_docking::GracefulController
 * @brief Default control law for approaching a dock target
 */
class GracefulController : public ControllerBase
{
public:
  /**
   * @brief Release the smooth control law along with the shared resources.
   */
  void cleanup() override;

  /**
   * @brief Activate parameter callbacks.
   */
  void activate() override;

  /**
   * @brief Deactivate the parameter callbacks.
   */
  void deactivate() override;

  /**
   * @brief Declare the smooth control law parameters and construct the control law.
   * @param node Lifecycle node
   */
  void onConfigure(const nav2::LifecycleNode::SharedPtr & node) override;

  /**
   * @brief Compute a velocity command using the smooth control law.
   * @param target Target pose, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @param dt Control loop duration [s]. Unused: the smooth control law is a pure feedback law.
   * @returns Command velocity.
   */
  geometry_msgs::msg::Twist computeCommand(
    const geometry_msgs::msg::Pose & target, bool reverse, double dt) override;

  /**
   * @brief Forward-simulate one step of the smooth control law.
   * @param dt Simulation time step [s].
   * @param target Target pose, in robot centric coordinates.
   * @param current Current pose along the projection, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @returns The pose one step further along the projection.
   */
  geometry_msgs::msg::Pose predictNextPose(
    double dt, const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current, bool reverse) override;

protected:
  /**
   * @brief Copy the latest parameters. Call with the param mutex held.
   */
  void applyParameters();

  // Smooth control law
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
  std::unique_ptr<GracefulParameterHandler> graceful_param_handler_;
  GracefulParameters * graceful_params_{nullptr};
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_PLUGINS__GRACEFUL_CONTROLLER_HPP_
