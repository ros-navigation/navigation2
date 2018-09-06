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

ExecuteMissionBehaviorTree::ExecuteMissionBehaviorTree(rclcpp::Node * node)
: node_(node)
{
  // Create the input message
  navigateToPoseCommand_ = std::make_shared<nav2_tasks::NavigateToPoseCommand>();

  // Create the nodes of the tree
  root_ = new BT::SequenceNodeWithMemory("Sequence");

  // Add the nodes to the tree, creating the tree structure
  root_->AddChild(navigateToPoseAction1_);
  root_->AddChild(navigateToPoseAction2_);
}

nav2_tasks::TaskStatus
ExecuteMissionBehaviorTree::run(nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  navigateToPoseCommand_ = command;

  rclcpp::WallRate loop_rate(100ms);
  BT::ReturnStatus result = root_->get_status();

  while (rclcpp::ok() && !(result == BT::SUCCESS || result == BT::FAILURE)) {
    result = root_->Tick();
    loop_rate.sleep();
  }

  return (result ==
         BT::SUCCESS) ? nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_mission_executor
