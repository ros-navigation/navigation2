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

#include "nav2_behavior_tree/plugins/action/controller_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

ControllerSelector::ControllerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();

  // Spin multiple times due to rclcpp regression in Jazzy requiring a 'warm up' spin
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

void ControllerSelector::initialize()
{
  createROSInterfaces();
}

void ControllerSelector::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !controller_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    controller_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_,
      std::bind(&ControllerSelector::callbackControllerSelect, this, _1),
      nav2::qos::LatchedTopicQoS(),
      callback_group_);
  }
}

BT::NodeStatus ControllerSelector::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_some();

  // This behavior always use the last selected controller received from the topic input.
  // When no input is specified it uses the default controller.
  // If the default controller is not specified then we work in "required controller mode":
  // In this mode, the behavior returns failure if the controller selection is not received from
  // the topic input.
  if (last_selected_controller_.empty()) {
    std::string default_controller;
    getInput("default_controller", default_controller);
    if (default_controller.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_controller_ = default_controller;
    }
  }

  setOutput("selected_controller", last_selected_controller_);

  return BT::NodeStatus::SUCCESS;
}

void
ControllerSelector::callbackControllerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_controller_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ControllerSelector>("ControllerSelector");
}
