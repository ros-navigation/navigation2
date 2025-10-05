// Copyright (c) 2025 Maurice Alexander Purnawan
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

#include "nav2_behavior_tree/plugins/action/path_handler_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PathHandlerSelector::PathHandlerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();
  bt_loop_duration_ =
    config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
}

void PathHandlerSelector::initialize()
{
  createROSInterfaces();
}

void PathHandlerSelector::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !path_handler_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    path_handler_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_,
      std::bind(&PathHandlerSelector::callbackPathHandlerSelect, this, _1),
      nav2::qos::LatchedSubscriptionQoS(),
      callback_group_);
  }
}

BT::NodeStatus PathHandlerSelector::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_all(bt_loop_duration_);

  // This behavior always use the last selected path_handler received from the topic input.
  // When no input is specified it uses the default path handler.
  // If the default path handler is not specified then we work in "required path handler mode":
  // In this mode, the behavior returns failure if the path handler selection is not received from
  // the topic input.
  if (last_selected_path_handler_.empty()) {
    std::string default_path_handler;
    getInput("default_path_handler", default_path_handler);
    if (default_path_handler.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_path_handler_ = default_path_handler;
    }
  }

  setOutput("selected_path_handler", last_selected_path_handler_);

  return BT::NodeStatus::SUCCESS;
}

void
PathHandlerSelector::callbackPathHandlerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_path_handler_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathHandlerSelector>("PathHandlerSelector");
}
