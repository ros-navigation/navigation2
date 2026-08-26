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

#ifndef OPENNAV_DOCKING__GRACEFUL_CONTROLLER_HPP_
#define OPENNAV_DOCKING__GRACEFUL_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "opennav_docking/controller_base.hpp"

namespace opennav_docking
{

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

protected:
  /**
   * @brief Declare the smooth control law parameters and construct the control law.
   * @param node Lifecycle node
   */
  void configureController(const nav2::LifecycleNode::SharedPtr & node) override;

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

  /**
   * @brief Apply a dynamic parameter update to the smooth control law.
   * @param name The parameter name with this controller's namespace prefix stripped.
   * @param parameter The parameter being updated.
   */
  void updateParameter(const std::string & name, const rclcpp::Parameter & parameter) override;

  // Smooth control law
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_, deceleration_max_, v_linear_min_, v_linear_max_, v_angular_max_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__GRACEFUL_CONTROLLER_HPP_
