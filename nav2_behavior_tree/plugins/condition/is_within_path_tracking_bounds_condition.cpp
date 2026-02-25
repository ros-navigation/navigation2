// Copyright (c) 2025 Berkan Tali
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

#include "nav2_behavior_tree/plugins/condition/is_within_path_tracking_bounds_condition.hpp"

namespace nav2_behavior_tree
{

IsWithinPathTrackingBoundsCondition::IsWithinPathTrackingBoundsCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  logger_ = node->get_logger();
  clock_ = node->get_clock();
}

void IsWithinPathTrackingBoundsCondition::initialize()
{
  getInput<double>("max_error_left", max_error_left_);
  getInput<double>("max_error_right", max_error_right_);
  getInput<double>("max_error_heading", max_error_heading_);
  if (max_error_right_ < 0.0) {
    RCLCPP_WARN_ONCE(logger_, "max_error_right should be positive, using absolute value");
    max_error_right_ = std::abs(max_error_right_);
  }
  if (max_error_left_ < 0.0) {
    RCLCPP_WARN_ONCE(logger_, "max_error_left should be positive, using absolute value");
    max_error_left_ = std::abs(max_error_left_);
  }
  if (max_error_heading_ < 0.0) {
    RCLCPP_WARN_ONCE(logger_, "max_error_heading should be positive, using absolute value");
    max_error_heading_ = std::abs(max_error_heading_);
  }
}

BT::NodeStatus IsWithinPathTrackingBoundsCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  getInput<nav2_msgs::msg::TrackingFeedback>("tracking_feedback", tracking_feedback_);
  double position_tracking_error = tracking_feedback_.position_tracking_error;
  double heading_tracking_error = tracking_feedback_.heading_tracking_error;
  bool is_within_position_bounds;
  if (position_tracking_error >= 0.0) {  // Positive or zero = left side (or on path)
    is_within_position_bounds = (position_tracking_error <= max_error_left_);
  } else {  // Negative = right side
    is_within_position_bounds = (std::abs(position_tracking_error) <= max_error_right_);
  }

  bool is_within_heading_bounds = std::abs(heading_tracking_error) <= max_error_heading_;

  if (is_within_position_bounds && is_within_heading_bounds) {
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      1000,
      "Robot is out of path tracking bounds! Position error: %.2f, Heading error: %.2f",
      position_tracking_error, heading_tracking_error);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
    "IsWithinPathTrackingBounds");
}
