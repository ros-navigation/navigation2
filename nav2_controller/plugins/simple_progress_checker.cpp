// Copyright (c) 2019 Intel Corporation
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

#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{
void SimpleProgressChecker::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  radius_ = node->declare_or_get_parameter(plugin_name + ".required_movement_radius", 0.5);
  time_allowance_ = rclcpp::Duration::from_seconds(node->declare_or_get_parameter(plugin_name +
      ".movement_time_allowance", 10.0));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleProgressChecker::dynamicParametersCallback, this, _1));
}

bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  if ((!baseline_pose_set_) || (isRobotMovedEnough(current_pose.pose))) {
    resetBaselinePose(current_pose.pose);
    return true;
  }
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

void SimpleProgressChecker::resetBaselinePose(const geometry_msgs::msg::Pose & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

bool SimpleProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_;
}

double SimpleProgressChecker::pose_distance(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2)
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;

  return std::hypot(dx, dy);
}

rcl_interfaces::msg::SetParametersResult
SimpleProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".required_movement_radius") {
        radius_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".movement_time_allowance") {
        time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleProgressChecker, nav2_core::ProgressChecker)
