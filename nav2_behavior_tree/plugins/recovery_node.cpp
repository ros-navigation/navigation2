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

#include "nav2_behavior_tree/recovery_node.hpp"

#include <string>

#include "behaviortree_cpp/bt_factory.h"


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}

namespace nav2_behavior_tree
{
RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf), current_child_idx_(0), retry_count_(0)
{
  getInput("number_of_retries", number_of_retries_);
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  retry_count_ = 0;
}

BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ < number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            retry_count_ = 0;
            halt();
            return BT::NodeStatus::SUCCESS;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            // tick second child
            if (retry_count_ <= number_of_retries_) {
              current_child_idx_++;
              break;
            } else {
              haltChildren(0);
              return BT::NodeStatus::FAILURE;
            }
          }
          break;

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }
          break;

        default:
          {
          }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            retry_count_++;
            current_child_idx_--;
            haltChildren(1);
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            current_child_idx_--;
            retry_count_ = 0;
            halt();
            return BT::NodeStatus::FAILURE;
          }
          break;

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }
          break;

        default:
          {
          }
      }  // end switch
    }
  }  // end while loop
  retry_count_ = 0;
  halt();
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree
