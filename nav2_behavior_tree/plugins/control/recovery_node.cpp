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

#ifndef NAV2_BEHAVIOR_TREE__RECOVERY_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__RECOVERY_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace nav2_behavior_tree
{
/**
 * @brief The RecoveryNode has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 *
 */
class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::ControlNode::ControlNode(name, conf), current_child_idx_(0), retry_count_(0)
  {
    getInput("number_of_retries", number_of_retries_);
  }

  ~RecoveryNode() override = default;

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 1, "Number of retries")
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;
  BT::NodeStatus tick() override
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

  void halt() override
  {
    ControlNode::haltChildren(1);
    current_child_idx_ = 0;
    retry_count_ = 0;
  }
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}

#endif  // NAV2_BEHAVIOR_TREE__RECOVERY_NODE_HPP_
