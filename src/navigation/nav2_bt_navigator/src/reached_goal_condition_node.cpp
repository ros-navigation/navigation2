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

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav2_bt_navigator/reached_goal_condition_node.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

ReachedGoalConditionNode::ReachedGoalConditionNode(rclcpp::Node::SharedPtr node)
: ConditionNode("ReachedGoalConditionNode"), taskClient_(node)
{
  result_ = std::make_shared<nav2_tasks::FollowPathResult>();
}

BT::ReturnStatus 
ReachedGoalConditionNode::Tick()
{
  // Quickly get the current status of the task so we can report back
  TaskStatus status = taskClient_.waitForResult(result_, std::chrono::milliseconds(1));

  if (status == TaskStatus::SUCCEEDED) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ReachedGoalConditionNode: succeeded");
  }

  // A condition node returns only success or failure, depending on whether
  // the condition had been satisfied
  return (status == TaskStatus::SUCCEEDED)? BT::SUCCESS : BT::FAILURE;
}

}  // namespace nav2_bt_navigator
