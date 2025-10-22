// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef NAV2_GRACEFUL_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_GRACEFUL_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_graceful_controller
{

struct Parameters
{
  double min_lookahead;
  double max_lookahead;
  double k_phi;
  double k_delta;
  double beta;
  double lambda;
  double v_linear_min;
  double v_linear_max;
  double v_linear_max_initial;
  double v_angular_max;
  double v_angular_max_initial;
  double v_angular_min_in_place;
  double slowdown_radius;
  bool initial_rotation;
  double initial_rotation_tolerance;
  bool prefer_final_rotation;
  double rotation_scaling_factor;
  bool allow_backward;
  double in_place_collision_resolution;
  bool use_collision_detection;
};

/**
 * @class nav2_graceful_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for GracefulMotionController
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::ParameterHandler
   */
  ParameterHandler(
    nav2::LifecycleNode::SharedPtr node,
    std::string & plugin_name,
    rclcpp::Logger & logger);

  /**
   * @brief Destructor for nav2_graceful_controller::ParameterHandler
   */
  ~ParameterHandler();

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  nav2::LifecycleNode::WeakPtr node_;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  Parameters params_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("GracefulMotionController")};
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__PARAMETER_HANDLER_HPP_
