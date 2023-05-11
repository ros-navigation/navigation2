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

#include <string>

#include "nav2_behavior_tree/plugins/condition/is_docked_condition.hpp"

namespace nav2_behavior_tree
{

IsDockedCondition::IsDockedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  dock_state_topic_("/dock_status"),
  is_docked_(false)
{
  getInput("dock_state_topic", dock_state_topic_);
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  dock_state_sub_ = node->create_subscription<nav2_msgs::msg::DockState>(
    dock_state_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsDockedCondition::dockCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsDockedCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_docked_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsDockedCondition::dockCallback(nav2_msgs::msg::DockState::SharedPtr msg)
{
  is_docked_ = msg->docked;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsDockedCondition>("IsDockedCondition");
}
