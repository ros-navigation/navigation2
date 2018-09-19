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

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "BTpp/behavior_tree.h"
#include "nav2_tasks/compute_path_to_pose_action.hpp"
#include "nav2_tasks/follow_path_action.hpp"
#include "nav2_tasks/navigate_to_pose_action.hpp"

namespace nav2_bt_navigator
{

class NavigateToPoseBehaviorTree
{
public:
  explicit NavigateToPoseBehaviorTree(rclcpp::Node::SharedPtr node);
  NavigateToPoseBehaviorTree() = delete;

  nav2_tasks::TaskStatus run(
    nav2_tasks::NavigateToPoseCommand::SharedPtr command,
    std::function<bool()> cancelRequested
    /*, loop rate*/);

private:
  // The ROS node to use for any task clients
  rclcpp::Node::SharedPtr node_;

  // The root node of the behavior tree
  std::unique_ptr<BT::SequenceNodeWithMemory> root_;

  // The actions that will be composed into a tree
  std::unique_ptr<nav2_tasks::ComputePathToPoseAction> computePathToPoseAction_;
  std::unique_ptr<nav2_tasks::FollowPathAction> followPathAction_;

  // The commands and results for each action
  nav2_tasks::ComputePathToPoseCommand::SharedPtr computePathToPoseCommand_;
  nav2_tasks::ComputePathToPoseResult::SharedPtr computePathToPoseResult_;
  nav2_tasks::FollowPathCommand::SharedPtr followPathCommand_;
  nav2_tasks::FollowPathResult::SharedPtr followPathResult_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATE_TO_POSE_BEHAVIOR_TREE_HPP_
