/******************************************************************************
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/condition/is_following_path_condition.hpp"
#include "nav2_msgs/msg/tracking_error.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

// ---------------------------------------------------------------------------
//  CTOR / DTOR
// ---------------------------------------------------------------------------
IsFollowingPathCondition::IsFollowingPathCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{}

IsFollowingPathCondition::~IsFollowingPathCondition() = default;

// ---------------------------------------------------------------------------
//  Lazy one-shot subscription initialiser
// ---------------------------------------------------------------------------
void IsFollowingPathCondition::initSubscriber()
{
  if (sub_ready_) {
    return;
  }

  // ---- read ports (defaults set in header) ----
  getInput("error_topic", topic_);
  getInput("max_error",  max_error_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  exec_.add_callback_group(cb_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = cb_group_;

  sub_ = node_->create_subscription<nav2_msgs::msg::TrackingError>(
    topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsFollowingPathCondition::errorCallback, this, std::placeholders::_1),
    opts);

  sub_ready_ = true;
}

// ---------------------------------------------------------------------------
//  Callback
// ---------------------------------------------------------------------------
void IsFollowingPathCondition::errorCallback(
  const nav2_msgs::msg::TrackingError::SharedPtr msg)
{
  last_error_  = msg->tracking_error;
  got_msg_     = true;
}

// ---------------------------------------------------------------------------
//  TICK
// ---------------------------------------------------------------------------
BT::NodeStatus IsFollowingPathCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initSubscriber();
  }

  exec_.spin_some();

  if (!got_msg_) {
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for first TrackingError message…");
    return BT::NodeStatus::FAILURE;
  }

  // Allow runtime update of the threshold
  getInput("max_error", max_error_);

  if (last_error_ <= max_error_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Tracking OK: error %.3f ≤ max %.3f", last_error_, max_error_);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "Tracking diverged: error %.3f > max %.3f", last_error_, max_error_);
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

// ---------------------------------------------------------------------------
//  BT registration
// ---------------------------------------------------------------------------
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsFollowingPathCondition>(
    "IsFollowingPath");
}
