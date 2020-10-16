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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace nav2_behavior_tree
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void NavigateToPoseAction::on_tick()
{
  // Use the position and orientation fields from the XML attributes to initialize the goal
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;

  if (!getInput("position", position) || !getInput("orientation", orientation)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: position or orientation not provided");
    return;
  }

  goal_.pose.pose.position = position;
  goal_.pose.pose.orientation = orientation;
}

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
