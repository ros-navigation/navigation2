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

#include "nav2_behavior_tree/plugins/control/sequence_with_blackboard_memory.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_behavior_tree
{

SequenceWithBlackboardMemoryNode::SequenceWithBlackboardMemoryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf) {}

BT::NodeStatus SequenceWithBlackboardMemoryNode::tick()
{
  const int children_count = children_nodes_.size();

  int current_child_idx;
  getInput("current_child_idx", current_child_idx);

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx < children_count) {
    TreeNode * current_child_node = children_nodes_[current_child_idx];
    const BT::NodeStatus child_status = current_child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
          return child_status;

      case BT::NodeStatus::FAILURE:
          // Reset on failure
          resetChildren();
          current_child_idx = 0;
          setOutput("current_child_idx", 0);
          return child_status;

      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::SKIPPED:
          // Skip the child node
          current_child_idx++;
          setOutput("current_child_idx", current_child_idx);
        break;

      case BT::NodeStatus::IDLE:
          throw std::runtime_error("A child node must never return IDLE");
    }  // end switch
  }  // end while loop

  // The entire while loop completed. This means that all the children returned SUCCESS.
  if (current_child_idx == children_count) {
    resetChildren();
    setOutput("current_child_idx", 0);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SequenceWithBlackboardMemoryNode>(
        name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::SequenceWithBlackboardMemoryNode>(
    "SequenceWithBlackboardMemory", builder);
}
