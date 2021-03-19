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
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{
static double pose_distance(const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);

void SimpleProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&SimpleProgressChecker::on_parameter_event_callback, this, _1));
}

bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose2d))) {
    reset_baseline_pose(current_pose2d);
    return true;
  }
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

void SimpleProgressChecker::reset_baseline_pose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

bool SimpleProgressChecker::is_robot_moved_enough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_;
}

static double pose_distance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  double dx = pose1.x - pose2.x;
  double dy = pose1.y - pose2.y;

  return std::hypot(dx, dy);
}

void
SimpleProgressChecker::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_radius") {
        radius_ = value.double_value;
      } else if (name == plugin_name_ + ".movement_time_allowance") {
        time_allowance_ = rclcpp::Duration::from_seconds(value.double_value);
      }
    }
  }
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleProgressChecker, nav2_core::ProgressChecker)
