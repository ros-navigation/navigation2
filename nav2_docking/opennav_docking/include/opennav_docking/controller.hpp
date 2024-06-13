// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING__CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::Controller
 * @brief Default control law for approaching a dock target
 */
class Controller
{
public:
  /**
   * @brief Create a controller instance. Configure ROS 2 parameters.
   */
  explicit Controller(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Compute a velocity command using control law.
   * @param pose Target pose, in robot centric coordinates.
   * @param cmd Command velocity.
   * @param backward If true, robot will drive backwards to goal.
   * @returns True if command is valid, false otherwise.
   */
  bool computeVelocityCommand(
    const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd,
    bool backward = false);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

protected:
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;

  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_, v_linear_min_, v_linear_max_, v_angular_max_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_HPP_
