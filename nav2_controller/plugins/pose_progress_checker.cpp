// Copyright (c) 2023 Dexory
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

#include <cmath>
#include <string>
#include <memory>
#include <vector>

#include "angles/angles.h"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav2_core/controller_exceptions.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

PoseProgressChecker::ParameterHandler::ParameterHandler(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string & plugin_name, rclcpp::Logger & logger)
{
  node_ = node;
  plugin_name_ = plugin_name;
  logger_ = logger;

  declare_parameter_if_not_declared(node, plugin_name + ".required_movement_angle",
    rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name + ".required_movement_angle",
    params_.required_movement_angle);
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    [this](const auto & params) {
      return this->validateParameterUpdatesCallback(params);
    });
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    [this](const auto & params) {
      return this->updateParametersCallback(params);
    });
}
PoseProgressChecker::ParameterHandler::~ParameterHandler()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}
void PoseProgressChecker::ParameterHandler::updateParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (name == plugin_name_ + ".required_movement_angle" &&
      type == ParameterType::PARAMETER_DOUBLE)
    {
      params_.required_movement_angle = param.as_double();
    }
  }
}
rcl_interfaces::msg::SetParametersResult
PoseProgressChecker::ParameterHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();
    {
      if (name.find(plugin_name_ + ".") != 0) {
        continue;
      }
      if (name == plugin_name_ + ".required_movement_angle" &&
        type == ParameterType::PARAMETER_DOUBLE &&
        (parameter.as_double() <= 0.0 || parameter.as_double() >= 2 * M_PI))
      {
        result.successful = false;
        result.reason = "The value required_movement_angle is incorrectly set, "
          "it should be 0 < required_movement_angle < 2PI. Ignoring parameter update.";
        return result;
      }
    }
  }
  result.successful = true;
  return result;
}

void PoseProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }
  node_ = parent;
  plugin_name_ = plugin_name;
  logger_ = node->get_logger();
  param_handler_ = std::make_unique<ParameterHandler>(node, plugin_name_, logger_);
  params_ = param_handler_->getParams();
  SimpleProgressChecker::initialize(parent, plugin_name);
}

bool PoseProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if (!baseline_pose_set_ || PoseProgressChecker::isRobotMovedEnough(current_pose2d)) {
    resetBaselinePose(current_pose2d);
    return true;
  }
  return clock_->now() - baseline_time_ <= time_allowance_;
}

bool PoseProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_ ||
         poseAngleDistance(pose, baseline_pose_) > params_->required_movement_angle;
}

double PoseProgressChecker::poseAngleDistance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  return abs(angles::shortest_angular_distance(pose1.theta, pose2.theta));
}


}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::PoseProgressChecker, nav2_core::ProgressChecker)
