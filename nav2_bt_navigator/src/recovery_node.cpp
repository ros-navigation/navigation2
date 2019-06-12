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
#include "nav2_bt_navigator/recovery_node.hpp"

namespace nav2_bt_navigator
{
RecoveryNode::RecoveryNode(const std::string & name, const BT::NodeParameters & params)
: BT::ControlNode::ControlNode(name, params), current_child_idx_(0), retry_count_(0)
{
  getParam<unsigned int>("num_of_retries", num_of_retries_);
}

BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count > 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ < num_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::SUCCESS:
        {
          if (current_child_idx_ == 0) {
            haltChildren(0);
            return BT::NodeStatus::SUCCESS;
          }
          if (current_child_idx_ == 1) {
            retry_count_++;
            current_child_idx_--;   //  retry first child
            haltChildren(1);   // Is this necessary?
          }
        }
        break;

      case BT::NodeStatus::FAILURE:
        {
          if (current_child_idx_ == 0) {
            if (retry_count_ <= num_of_retries_) {
              current_child_idx_++;
              break;
            } else {
              haltChildren(0);
              return BT::NodeStatus::FAILURE;
            }
          }
          if (current_child_idx_ == 1) {
            ControlNode::halt();
            return BT::NodeStatus::FAILURE;
          }
        }
        break;

      case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;  // Is this necessary?
        }
        break;

      default:
        {
          // throw?
        }
    }  // end switch
  }  // end while loop
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_navigator
