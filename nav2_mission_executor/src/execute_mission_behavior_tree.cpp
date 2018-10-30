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

#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

#include <memory>
#include <string>
#include <set>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "behavior_tree_core/xml_parsing.h"
#include "Blackboard/blackboard_local.h"
#include "nav2_tasks/navigate_to_pose_action.hpp"
#include "nav2_tasks/bt_conversions.hpp"

using namespace std::chrono_literals;

namespace nav2_mission_executor
{

ExecuteMissionBehaviorTree::ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Register our custom action nodes so that they can be included in XML description
  factory_.registerNodeType<nav2_tasks::NavigateToPoseAction>("NavigateToPose");
}

nav2_tasks::TaskStatus ExecuteMissionBehaviorTree::run(
  const std::string & behavior_tree_xml,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds tree_tick_timeout)
{
  // Create the blackboard that will be shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Set a couple values that all of the action nodes expect/require
  blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
  blackboard->set<std::chrono::milliseconds>("node_loop_timeout", std::chrono::milliseconds(100));

  // The complete behavior tree that results from parsing the incoming XML. When the tree goes
  // out of scope, all the nodes are destroyed
  BT::Tree tree = BT::buildTreeFromText(factory_, behavior_tree_xml, blackboard);

  rclcpp::WallRate loop_rate(tree_tick_timeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Loop until something happens with ROS or the node completes w/ success or failure
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree.root_node->executeTick();

    // Check if we've received a cancel message
    if (cancelRequested()) {
      tree.root_node->halt();
      return nav2_tasks::TaskStatus::CANCELED;
    }

    loop_rate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ?
         nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_mission_executor
