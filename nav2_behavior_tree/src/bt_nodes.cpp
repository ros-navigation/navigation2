// Copyright (c) 2019 Intel Corporation
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

#include "nav2_behavior_tree/bt_nodes.hpp"

#include "nav2_behavior_tree/back_up_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_behavior_tree/compute_path_to_pose_action.hpp"
#include "nav2_behavior_tree/follow_path_action.hpp"
#include "nav2_behavior_tree/goal_reached_condition.hpp"
#include "nav2_behavior_tree/is_stuck_condition.hpp"
#include "nav2_behavior_tree/rate_controller_node.hpp"
#include "nav2_behavior_tree/recovery_node.hpp"
#include "nav2_behavior_tree/spin_action.hpp"
#include "nav2_behavior_tree/wait_action.hpp"
#include "nav2_behavior_tree/clear_costmap_service.hpp"
#include "nav2_behavior_tree/reinitialize_global_localization_service.hpp"


BT_REGISTER_NODES(factory)
{
  nav2_behavior_tree::RegisterNodes(factory);
}

namespace nav2_behavior_tree
{

void RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  // Register our custom action nodes so that they can be included in XML description
  factory.registerNodeType<nav2_behavior_tree::ComputePathToPoseAction>("ComputePathToPose");
  factory.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");
  factory.registerNodeType<nav2_behavior_tree::BackUpAction>("BackUp");
  factory.registerNodeType<nav2_behavior_tree::SpinAction>("Spin");
  factory.registerNodeType<nav2_behavior_tree::WaitAction>("Wait");
  factory.registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  factory.registerNodeType<nav2_behavior_tree::ReinitializeGlobalLocalizationService>(
    "ReinitializeGlobalLocalization");

  // Register our custom condition nodes
  factory.registerNodeType<nav2_behavior_tree::IsStuckCondition>("IsStuck");
  factory.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");

  // Register our simple condition nodes
  factory.registerSimpleCondition("initialPoseReceived",
    std::bind(&nav2_behavior_tree::initialPoseReceived, std::placeholders::_1));

  // Register our custom decorator nodes
  factory.registerNodeType<nav2_behavior_tree::RateController>("RateController");

  // Register our custom control nodes
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}

BT::NodeStatus
initialPoseReceived(BT::TreeNode & tree_node)
{
  auto initPoseReceived = tree_node.config().blackboard->get<bool>("initial_pose_received");
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree
