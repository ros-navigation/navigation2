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

#include "rclcpp/rclcpp.hpp"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"
#include "nav2_tasks/compute_path_to_pose_action.hpp"
#include "nav2_tasks/follow_path_action.hpp"
#include "nav2_tasks/rate_controller_node.hpp"

using namespace std::chrono_literals;

namespace nav2_bt_navigator
{

NavigateToPoseBehaviorTree::NavigateToPoseBehaviorTree(rclcpp::Node::SharedPtr node)
: BehaviorTreeEngine(node)
{
  // Register our custom action nodes so that they can be included in XML description
  factory_.registerNodeType<nav2_tasks::ComputePathToPoseAction>("ComputePathToPose");
  factory_.registerNodeType<nav2_tasks::FollowPathAction>("FollowPath");

  // Register our custom decorator nodes
  factory_.registerNodeType<nav2_tasks::RateController>("RateController");

  // Register our Simple Action nodes
  factory_.registerSimpleAction("UpdatePath", std::bind(&NavigateToPoseBehaviorTree::updatePath, this, std::placeholders::_1));

  // The parallel node is not yet registered in the BehaviorTree.CPP library
  factory_.registerNodeType<BT::ParallelNode>("Parallel");

  task_client_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(node);
}

BT::NodeStatus NavigateToPoseBehaviorTree::updatePath(BT::TreeNode & tree_node)
{
  printf("NavigateToPoseBehaviorTree: updatePath\n");

  auto path = tree_node.blackboard()->template get<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path");

  int index = 0;
  for (auto pose : path->poses) {
    printf("point %u x: %0.2f, y: %0.2f\n", index, pose.position.x, pose.position.y);
    index++;
  }

  task_client_->sendPreempt(path);

  return BT::NodeStatus::RUNNING;
}

}  // namespace nav2_bt_navigator
