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

#ifndef NAV2_MISSION_EXECUTOR__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_
#define NAV2_MISSION_EXECUTOR__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behavior_tree_core/behavior_tree.h"
#include "behavior_tree_core/bt_factory.h"
#include "behavior_tree_core/xml_parsing.h"
#include "nav2_tasks/navigate_to_pose_action.hpp"

namespace nav2_mission_executor
{

class ExecuteMissionBehaviorTree
{
public:
  explicit ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node);
  ExecuteMissionBehaviorTree() = delete;
  ~ExecuteMissionBehaviorTree();

  nav2_tasks::TaskStatus run(
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(100));

private:
  // The ROS node to use for any task clients
  rclcpp::Node::SharedPtr node_;

  // A blackboard that is shared among all of the leaf nodes
  BT::Blackboard::Ptr blackboard_;

  // The complete behavior tree that results from parsing the incoming XML
  std::shared_ptr<BT::Tree> tree_;

  // The commands and results for each action
  nav2_tasks::NavigateToPoseCommand::SharedPtr navigateToPoseCommand_;
  nav2_tasks::NavigateToPoseResult::SharedPtr navigateToPoseResult_;

  // A factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace nav2_mission_executor

#endif  // NAV2_MISSION_EXECUTOR__EXECUTE_MISSION_BEHAVIOR_TREE_HPP_
