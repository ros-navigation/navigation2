// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PLANNER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PLANNER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

class PlannerSelector : public BT::DecoratorNode
{
public:
  PlannerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std_msgs::msg::String>(
        "selected_planner",
        "Selected planner by subscription"),
      BT::InputPort<std::string>(
        "default_planner",
        "the default planner to use if there is not any external topic message received.")
    };
  }

private:
  BT::NodeStatus tick() override;

  void callback_planner_select(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_selector_sub_;

  std_msgs::msg::String last_selected_planner_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PLANNER_SELECTOR_NODE_HPP_
