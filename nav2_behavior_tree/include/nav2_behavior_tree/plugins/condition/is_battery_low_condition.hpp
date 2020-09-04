// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  IsBatteryLowCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryLowCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "Battery topic"),
      BT::InputPort<bool>(
        "is_voltage", false, "If true voltage will be used to check for low battery"),
    };
  }

private:
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::string battery_topic_;
  double min_battery_;
  bool is_voltage_;
  bool is_battery_low_;
  std::mutex mutex_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
