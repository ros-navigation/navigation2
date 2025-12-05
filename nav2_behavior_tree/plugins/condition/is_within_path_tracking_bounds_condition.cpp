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
: BT::ConditionNode(condition_name, conf),
  last_error_(std::numeric_limits<double>::max())
{
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  logger_ = node->get_logger();
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  tracking_feedback_sub_ = node->create_subscription<nav2_msgs::msg::TrackingFeedback>(
    "tracking_feedback",
    std::bind(&IsWithinPathTrackingBoundsCondition::trackingFeedbackCallback, this,
      std::placeholders::_1),
    rclcpp::SystemDefaultsQoS(),
    callback_group_);

  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");

  RCLCPP_INFO(logger_, "Initialized IsWithinPathTrackingBoundsCondition BT node");
  initialize();
}

void IsWithinPathTrackingBoundsCondition::trackingFeedbackCallback(
  const nav2_msgs::msg::TrackingFeedback::SharedPtr msg)
{
  last_error_ = msg->tracking_error;
}

void IsWithinPathTrackingBoundsCondition::initialize()
{
  getInput("max_error_left", max_error_left_);
  getInput("max_error_right", max_error_right_);
}

BT::NodeStatus IsWithinPathTrackingBoundsCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_all(bt_loop_duration_);

  if (!getInput("max_error_left", max_error_left_)) {
    RCLCPP_ERROR(logger_, "max_error_left parameter not provided");
    return BT::NodeStatus::FAILURE;
  }

  if (max_error_left_ < 0.0) {
    RCLCPP_WARN(logger_, "max_error_left should be positive, using absolute value");
    max_error_left_ = std::abs(max_error_left_);
  }

  if (!getInput("max_error_right", max_error_right_)) {
    RCLCPP_ERROR(logger_, "max_error_right parameter not provided");
    return BT::NodeStatus::FAILURE;
  }

  if (max_error_right_ < 0.0) {
    RCLCPP_WARN(logger_, "max_error_right should be positive, using absolute value");
    max_error_right_ = std::abs(max_error_right_);
  }

  if (last_error_ == std::numeric_limits<double>::max()) {
    RCLCPP_WARN(logger_, "No tracking feedback received yet.");
    return BT::NodeStatus::FAILURE;
  }

  if (last_error_ > 0.0) {  // Positive = left side
    if (last_error_ > max_error_left_) {
      return BT::NodeStatus::FAILURE;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  } else {  // Negative = right side
    if (std::abs(last_error_) > max_error_right_) {
      return BT::NodeStatus::FAILURE;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
    "IsWithinPathTrackingBounds");
}
