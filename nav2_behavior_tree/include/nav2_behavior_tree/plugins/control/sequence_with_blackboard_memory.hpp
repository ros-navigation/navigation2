// Copyright (c) 2025 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__SEQUENCE_WITH_BLACKBOARD_MEMORY_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__SEQUENCE_WITH_BLACKBOARD_MEMORY_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_behavior_tree
{
/**
 * @brief The SequenceWithBlackboardMemoryNode is similar to the SequenceNode, but it
 * stores the index of the last running child in the blackboard (key: "current_child_idx"),
 * and it does not reset the index when it got halted.
 * It used to tick children in an ordered sequence. If any child returns RUNNING, previous
 * children will NOT be ticked again.
 *
 * - If all the children return SUCCESS, this node returns SUCCESS.
 *
 * - If a child returns RUNNING, this node returns RUNNING.
 *   Loop is NOT restarted, the same running child will be ticked again.
 *
 * - If a child returns FAILURE, stop the loop and return FAILURE.
 *   Restart the loop only if (reset_on_failure == true)
 *
 */
class SequenceWithBlackboardMemoryNode : public BT::ControlNode
{
public:
  SequenceWithBlackboardMemoryNode(const std::string & name, const BT::NodeConfiguration & conf);

  ~SequenceWithBlackboardMemoryNode() override = default;

  //! @brief Declare ports
  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<int>("current_child_idx", "The index of the current child"),
    };
  }

private:
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__SEQUENCE_WITH_BLACKBOARD_MEMORY_HPP_
