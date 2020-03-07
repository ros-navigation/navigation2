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

#ifndef NAV2_BEHAVIOR_TREE__FOLLOW_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__FOLLOW_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
  {
    config().blackboard->set("path_updated", false);
  }

  void on_tick() override
  {
    getInput("path", goal_.path);
    getInput("controller_id", goal_.controller_id);
  }

  void on_wait_for_result() override
  {
    // Check if the goal has been updated
    if (config().blackboard->get<bool>("path_updated")) {
      // Reset the flag in the blackboard
      config().blackboard->set("path_updated", false);  // NOLINT

      // Grab the new goal and set the flag so that we send the new goal to
      // the action server on the next loop iteration
      getInput("path", goal_.path);
      goal_updated_ = true;
    }
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
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
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}

#endif  // NAV2_BEHAVIOR_TREE__FOLLOW_PATH_ACTION_HPP_
