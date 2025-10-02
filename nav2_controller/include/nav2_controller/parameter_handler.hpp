// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_controller
{

struct Parameters
{
  double controller_frequency;
  double transform_tolerance;
  double min_x_velocity_threshold;
  double min_y_velocity_threshold;
  double min_theta_velocity_threshold;
  std::string speed_limit_topic;
  double failure_tolerance;
  bool use_realtime_priority;
  bool publish_zero_velocity;
  double costmap_update_timeout;
  std::string odom_topic;
  double odom_duration;
  bool interpolate_curvature_after_goal;
  double max_robot_pose_search_dist;
  double prune_distance;
  bool enforce_path_inversion;
  float inversion_xy_tolerance;
  float inversion_yaw_tolerance;
  std::vector<std::string> progress_checker_ids;
  std::vector<std::string> progress_checker_types;
  std::vector<std::string> goal_checker_ids;
  std::vector<std::string> goal_checker_types;
  std::vector<std::string> controller_ids;
  std::vector<std::string> controller_types;
};

/**
 * @class nav2_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
  /**
   * @brief Constructor for nav2_controller::ParameterHandler
   */
  ParameterHandler(
    nav2::LifecycleNode::SharedPtr node,
    const rclcpp::Logger & logger, const double costmap_size_x);

  /**
   * @brief Destrructor for nav2_controller::ParameterHandler
   */
  ~ParameterHandler();

  std::mutex & getMutex() {return mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  nav2::LifecycleNode::WeakPtr node_;
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  void
  updateParametersCallback(std::vector<rclcpp::Parameter> parameters);
  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(std::vector<rclcpp::Parameter> parameters);
  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  Parameters params_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("GracefulMotionController")};

  const std::vector<std::string> default_progress_checker_ids_{"progress_checker"};
  const std::vector<std::string> default_progress_checker_types_{
    "nav2_controller::SimpleProgressChecker"};
  const std::vector<std::string> default_goal_checker_ids_{"goal_checker"};
  const std::vector<std::string> default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"};
  const std::vector<std::string> default_controller_ids_{"FollowPath"};
  const std::vector<std::string> default_controller_types_{"dwb_core::DWBLocalPlanner"};
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PARAMETER_HANDLER_HPP_
