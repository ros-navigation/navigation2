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

#include "nav2_behavior_tree/plugins/condition/is_battery_charging_condition.hpp"

namespace nav2_behavior_tree
{

IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  is_battery_charging_(false)
{
  getInput("battery_topic", battery_topic_);
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1),
    sub_option);

  // Spin multiple times due to rclcpp regression in Jazzy requiring a 'warm up' spin
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_battery_charging_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  is_battery_charging_ =
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryChargingCondition>("IsBatteryCharging");
}
