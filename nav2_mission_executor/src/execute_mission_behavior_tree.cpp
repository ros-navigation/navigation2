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
#include <thread>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "Blackboard/blackboard_local.h"
#include "behavior_tree_core/xml_parsing.h"

using namespace std::chrono_literals;

namespace nav2_mission_executor
{

ExecuteMissionBehaviorTree::ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Register nodes that may occur in the behavior tree XML description
  factory_.registerNodeType<nav2_tasks::NavigateToPoseAction>("NavigateToPoseAction");

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  // Set a couple values that all of the action nodes expect/require
  blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  blackboard_->set<std::chrono::milliseconds>("tick_timeout", std::chrono::milliseconds(100));
}

nav2_tasks::TaskStatus ExecuteMissionBehaviorTree::run(
  std::function<bool()> cancelRequested, std::string & xml_text, std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // When the tree goes out of scope, all the nodes are destroyed
  tree_ = BT::buildTreeFromText(factory_, xml_text, blackboard_);

  // Loop until something happens with ROS or the node completes w/ success or failure
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING)
  {
    result = tree_->root_node->executeTick();

    // Check if we've received a cancel message
    if (cancelRequested()) {
      tree_->root_node->halt();
      return nav2_tasks::TaskStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ?
         nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_mission_executor
