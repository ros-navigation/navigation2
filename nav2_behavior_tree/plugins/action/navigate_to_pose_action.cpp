// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__NAVIGATE_TO_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__NAVIGATE_TO_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

class NavigateToPoseAction : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
    if (!getInput("goal", goal_.pose)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "NavigateToPoseAction: goal not provided");
      return;
    }
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
      });
  }
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", builder);
}

#endif  // NAV2_BEHAVIOR_TREE__NAVIGATE_TO_POSE_ACTION_HPP_
