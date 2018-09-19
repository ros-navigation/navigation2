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

#ifndef NAV2_MISSION_PLANNING__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_
#define NAV2_MISSION_PLANNING__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behavior_tree.h"
#include "nav2_tasks/execute_mission_task.hpp"
#include "nav2_tasks/navigate_to_pose_action.hpp"

namespace nav2_mission_executor
{

class ExecuteMissionBehaviorTree
{
public:
  explicit ExecuteMissionBehaviorTree(rclcpp::Node * node);
  ExecuteMissionBehaviorTree() = delete;

  nav2_tasks::TaskStatus run(
    std::function<bool ()> cancelRequested
    /*, loop rate*/);

private:
  // The ROS node to use for any task clients
  rclcpp::Node * node_;

  // The root node of the behavior tree
  std::unique_ptr<BT::SequenceNodeWithMemory> root_;

  // The actions that will be composed into a tree
  std::unique_ptr<nav2_tasks::NavigateToPoseAction> navigateToPoseAction1_;
  std::unique_ptr<nav2_tasks::NavigateToPoseAction> navigateToPoseAction2_;

  // The commands and results for each action
  nav2_tasks::NavigateToPoseCommand::SharedPtr navigateToPoseCommand_;
  nav2_tasks::NavigateToPoseResult::SharedPtr navigateToPoseResult_;
};

}  // namespace nav2_mission_executor

#endif //  NAV2_MISSION_PLANNING__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_
