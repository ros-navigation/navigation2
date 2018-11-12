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

using namespace std::chrono_literals;

namespace nav2_bt_navigator
{

NavigateToPoseBehaviorTree::NavigateToPoseBehaviorTree(rclcpp::Node::SharedPtr node)
: BehaviorTreeEngine(node)
{
  // Register our custom action nodes so that they can be included in XML description
  factory_.registerNodeType<nav2_tasks::ComputePathToPoseAction>("ComputePathToPose");
  factory_.registerNodeType<nav2_tasks::FollowPathAction>("FollowPath");
}

}  // namespace nav2_bt_navigator
