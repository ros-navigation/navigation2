// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
// Copyright (c) 2022 Owen Hooper
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

#include "nav2_behavior_tree/plugins/action/smoother_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

SmootherSelector::SmootherSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("topic_name", topic_name_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  smoother_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&SmootherSelector::callbackSmootherSelect, this, _1),
    sub_option);
}

BT::NodeStatus SmootherSelector::tick()
{
  callback_group_executor_.spin_some();

  // This behavior always use the last selected smoother received from the topic input.
  // When no input is specified it uses the default smoother.
  // If the default smoother is not specified then we work in "required smoother mode":
  // In this mode, the behavior returns failure if the smoother selection is not received from
  // the topic input.
  if (last_selected_smoother_.empty()) {
    std::string default_smoother;
    getInput("default_smoother", default_smoother);
    if (default_smoother.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_smoother_ = default_smoother;
    }
  }

  setOutput("selected_smoother", last_selected_smoother_);

  return BT::NodeStatus::SUCCESS;
}

void
SmootherSelector::callbackSmootherSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_smoother_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SmootherSelector>("SmootherSelector");
}
