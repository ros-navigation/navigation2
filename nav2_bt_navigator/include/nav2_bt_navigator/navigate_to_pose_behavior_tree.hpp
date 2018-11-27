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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATE_TO_POSE_BEHAVIOR_TREE_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATE_TO_POSE_BEHAVIOR_TREE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/behavior_tree_engine.hpp"
#include "nav2_tasks/follow_path_task.hpp"

namespace nav2_bt_navigator
{

class NavigateToPoseBehaviorTree : public nav2_tasks::BehaviorTreeEngine
{
public:
  explicit NavigateToPoseBehaviorTree(rclcpp::Node::SharedPtr node);
  NavigateToPoseBehaviorTree() = delete;

private:
  // Support for a BT SimpleActionNode that updates the FollowPath task
  BT::NodeStatus updatePath(BT::TreeNode & tree_node);
  std::unique_ptr<nav2_tasks::FollowPathTaskClient> follow_path_task_client_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATE_TO_POSE_BEHAVIOR_TREE_HPP_
