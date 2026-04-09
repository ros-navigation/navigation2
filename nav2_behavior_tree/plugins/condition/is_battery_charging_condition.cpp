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
  is_battery_charging_(false),
  is_global_(false),
  current_run_id_("")
{
  initialize();
  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
}

void IsBatteryChargingCondition::initialize()
{
  createROSInterfaces();
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  is_global_ = node->declare_or_get_parameter("is_global", false);
}

void IsBatteryChargingCondition::createROSInterfaces()
{
  std::string battery_topic_new;
  getInput("battery_topic", battery_topic_new);

  // Only create a new subscriber if the topic has changed or subscriber is empty
  if (battery_topic_new != battery_topic_ || !battery_sub_) {
    battery_topic_ = battery_topic_new;
    auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

    battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_,
      std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1),
      nav2::qos::StandardTopicQoS(),
      callback_group_);
  }
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  // Global mode: reinitialize (return FAILURE) when RunID changes
  if (is_global_) {
    std::string new_run_id;
    try {
      new_run_id = config().blackboard->template get<std::string>("run_id");
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "is_global=true requires 'run_id' to be set on the blackboard for condition: " +
        std::string(name()));
    }

    if (new_run_id != current_run_id_) {
      current_run_id_ = new_run_id;
      is_battery_charging_ = false;
    }
  }

  callback_group_executor_.spin_all(bt_loop_duration_);
  if (is_battery_charging_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(
  const sensor_msgs::msg::BatteryState::ConstSharedPtr & msg)
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
