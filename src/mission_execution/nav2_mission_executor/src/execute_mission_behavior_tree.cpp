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
#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

using namespace std::chrono_literals;

namespace nav2_mission_executor
{

ExecuteMissionBehaviorTree::ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Create the input and output messages
  navigateToPoseCommand_ = std::make_shared<nav2_tasks::NavigateToPoseCommand>();
  navigateToPoseResult_ = std::make_shared<nav2_tasks::NavigateToPoseResult>();

  // Compose the NavigateToPose message for the Navigation module. Fake out some values
  // for now. The goal pose would actually come from the Mission Plan. Could pass the mission
  // plan in the constructor and then use the values from there to instance each of the nodes.
  navigateToPoseCommand_->pose.position.x = 0;
  navigateToPoseCommand_->pose.position.y = 1;
  navigateToPoseCommand_->pose.position.z = 2;
  navigateToPoseCommand_->pose.orientation.x = 0;
  navigateToPoseCommand_->pose.orientation.y = 1;
  navigateToPoseCommand_->pose.orientation.z = 2;
  navigateToPoseCommand_->pose.orientation.w = 3;

  // Create the nodes of the tree
  root_ = std::make_unique<BT::SequenceNodeWithMemory>("Sequence");

  navigateToPoseAction1_ = std::make_unique<nav2_tasks::NavigateToPoseAction>(node_,
      "NavigateToPoseAction1", navigateToPoseCommand_, navigateToPoseResult_);
  navigateToPoseAction2_ = std::make_unique<nav2_tasks::NavigateToPoseAction>(node_,
      "NavigateToPoseAction2", navigateToPoseCommand_, navigateToPoseResult_);

  // Add the nodes to the tree, creating the tree structure
  root_->addChild(navigateToPoseAction1_.get());
  root_->addChild(navigateToPoseAction2_.get());
}

ExecuteMissionBehaviorTree::~ExecuteMissionBehaviorTree()
{
  BT::haltAllActions(root_.get());
}

nav2_tasks::TaskStatus
ExecuteMissionBehaviorTree::run(
  std::function<bool()> cancelRequested, std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = root_->status();

  while (rclcpp::ok() && !(result == BT::NodeStatus::SUCCESS || result == BT::NodeStatus::FAILURE)) {
    result = root_->executeTick();

    // Check if this task server has received a cancel message
    if (cancelRequested()) {
      root_->halt();
      return nav2_tasks::TaskStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result ==
         BT::NodeStatus::SUCCESS) ? nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_mission_executor
