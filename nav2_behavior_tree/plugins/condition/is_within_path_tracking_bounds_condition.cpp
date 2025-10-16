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
  last_error_(0.0)
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  tracking_feedback_sub_ = node_->create_subscription<nav2_msgs::msg::TrackingFeedback>(
    "/tracking_feedback",
    std::bind(&IsWithinPathTrackingBoundsCondition::trackingFeedbackCallback, this,
      std::placeholders::_1),
    rclcpp::SystemDefaultsQoS(),
    callback_group_);

  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");

  RCLCPP_DEBUG(node_->get_logger(), "Initialized IsWithinPathTrackingBoundsCondition BT node");
  RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting for tracking error");
  initialize();
}

void IsWithinPathTrackingBoundsCondition::trackingFeedbackCallback(
  const nav2_msgs::msg::TrackingFeedback::SharedPtr msg)
{
  last_error_ = msg->tracking_error;
}

void IsWithinPathTrackingBoundsCondition::initialize()
{
  getInput("max_error", max_error_);
}

BT::NodeStatus IsWithinPathTrackingBoundsCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_all(bt_loop_duration_);

  if (!getInput("max_error", max_error_)) {
    RCLCPP_ERROR(node_->get_logger(), "max_error parameter not provided");
    max_error_ = 1.0;  // Default fallback
  }

  if (max_error_ < 0.0) {
    RCLCPP_WARN(node_->get_logger(), "max_error should be positive, using absolute value");
    max_error_ = std::abs(max_error_);
  }

  if (last_error_ <= max_error_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
    "IsWithinPathTrackingBounds");
}
