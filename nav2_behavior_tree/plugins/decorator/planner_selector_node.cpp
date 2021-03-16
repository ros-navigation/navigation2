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

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/planner_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PlannerSelector::PlannerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  last_selected_planner_.data = "";

  std::string planner_selector_topic;

  node->get_parameter_or<std::string>(
    "planner_selector_topic", planner_selector_topic,
    "planner_selector");

  planner_selector_sub_ = node->create_subscription<std_msgs::msg::String>(
    planner_selector_topic, 10, std::bind(&PlannerSelector::callback_planner_select, this, _1));
}

inline BT::NodeStatus PlannerSelector::tick()
{
  if (last_selected_planner_.data.empty()) {
    getInput("default_planner", last_selected_planner_.data);
  }

  setOutput("selected_planner", last_selected_planner_);

  return child_node_->executeTick();
}

void
PlannerSelector::callback_planner_select(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_planner_ = *msg;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PlannerSelector>("PlannerSelector");
}
