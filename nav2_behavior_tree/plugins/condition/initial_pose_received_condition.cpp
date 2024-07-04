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

#include "nav2_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{
InitialPoseReceived::InitialPoseReceived(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus InitialPoseReceived::tick()
{
  bool initPoseReceived = false;
  BT::getInputOrBlackboard("initial_pose_received", initPoseReceived);
  return initPoseReceived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::InitialPoseReceived>("InitialPoseReceived");
}
