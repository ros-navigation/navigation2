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

#include "nav2_behavior_tree/plugins/action/planner_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PlannerSelector::PlannerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();
}

void PlannerSelector::initialize()
{
  createROSInterfaces();
}

void PlannerSelector::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !planner_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    planner_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_,
      std::bind(&PlannerSelector::callbackPlannerSelect, this, _1),
      nav2::qos::LatchedSubscriptionQoS(),
      callback_group_);
  }
}

BT::NodeStatus PlannerSelector::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_all(std::chrono::milliseconds(50));

  // This behavior always use the last selected planner received from the topic input.
  // When no input is specified it uses the default planner.
  // If the default planner is not specified then we work in "required planner mode":
  // In this mode, the behavior returns failure if the planner selection is not received from
  // the topic input.
  if (last_selected_planner_.empty()) {
    std::string default_planner;
    getInput("default_planner", default_planner);
    if (default_planner.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_planner_ = default_planner;
    }
  }

  setOutput("selected_planner", last_selected_planner_);

  return BT::NodeStatus::SUCCESS;
}

void
PlannerSelector::callbackPlannerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_planner_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PlannerSelector>("PlannerSelector");
}
