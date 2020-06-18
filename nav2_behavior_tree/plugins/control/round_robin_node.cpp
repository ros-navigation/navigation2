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

#include <string>

#include "nav2_behavior_tree/plugins/control/round_robin_node.hpp"

namespace nav2_behavior_tree
{

RoundRobinNode::RoundRobinNode(const std::string & name)
: BT::ControlNode::ControlNode(name, {})
{
  setRegistrationID("RoundRobin");
}

BT::NodeStatus RoundRobinNode::tick()
{
  const unsigned num_children = children_nodes_.size();

  setStatus(BT::NodeStatus::RUNNING);

  TreeNode * child_node = children_nodes_[current_child_idx_];
  const BT::NodeStatus child_status = child_node->executeTick();

  switch (child_status) {
    case BT::NodeStatus::SUCCESS:
      // Wrap around to the first child
      if (++current_child_idx_ == num_children) {
        // TODO(mjeronimo): halt this child (not all children)
        current_child_idx_ = 0;
      }

      ControlNode::haltChildren();
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
      ControlNode::haltChildren();
      return BT::NodeStatus::FAILURE;

    case BT::NodeStatus::RUNNING:
      break;

    default:
      throw BT::LogicError("Invalid status return from BT node");
  }

  return BT::NodeStatus::RUNNING;
}

void RoundRobinNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RoundRobinNode>("RoundRobin");
}
