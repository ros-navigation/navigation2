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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__INITIAL_POSE_RECEIVED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__INITIAL_POSE_RECEIVED_CONDITION_HPP_

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{
/**
 * @brief A BT::ConditionNode that returns SUCCESS if initial pose
 * has been received and FAILURE otherwise
 */
class InitialPoseReceived : public BT::ConditionNode
{
public:
  InitialPoseReceived(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("initial_pose_received")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__INITIAL_POSE_RECEIVED_CONDITION_HPP_
