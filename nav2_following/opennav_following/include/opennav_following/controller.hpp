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

#ifndef OPENNAV_FOLLOWING__CONTROLLER_HPP_
#define OPENNAV_FOLLOWING__CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace opennav_following
{
/**
 * @class opennav_following::Controller
 * @brief Control law for approaching the followed object
 */
class Controller
{
public:
  /**
   * @brief Create a controller instance. Configure ROS 2 parameters.
   *
   * @param node Lifecycle node
   */
  explicit Controller(const nav2::LifecycleNode::SharedPtr & node);

  /**
   * @brief A destructor for opennav_following::Controller
   */
  ~Controller();

  /**
   * @brief Compute a velocity command using control law.
   * @param pose Target pose, in robot centric coordinates.
   * @param backward If true, robot will drive backwards to goal.
   * @returns Command velocity.
   */
  geometry_msgs::msg::Twist computeVelocityCommands(
    const geometry_msgs::msg::Pose & pose, bool backward = false);

  /**
   * @brief Perform a command for in-place rotation.
   * @param angular_distance_to_heading Angular distance to goal.
   * @param current_velocity Current angular velocity.
   * @param dt Control loop duration [s].
   * @returns TwistStamped command for in-place rotation.
   */
  geometry_msgs::msg::Twist computeRotateToHeadingCommand(
    const double & angular_distance_to_heading,
    const geometry_msgs::msg::Twist & current_velocity,
    const double & dt);

protected:
  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  std::mutex dynamic_params_lock_;

  rclcpp::Logger logger_{rclcpp::get_logger("Controller")};

  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_, deceleration_max_, v_linear_min_, v_linear_max_, v_angular_max_;
  double rotate_to_heading_angular_vel_, rotate_to_heading_max_angular_accel_;
};

}  // namespace opennav_following

#endif  // OPENNAV_FOLLOWING__CONTROLLER_HPP_
