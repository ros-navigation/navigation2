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

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

namespace nav2_behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0)
{
  getInput("reset_distance", reset_distance_);
  if (reset_distance_ > 0.0) {
    getInput("robot_frame", robot_frame_);
    getInput("global_frame", global_frame_);
    const auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
    clock_ = node->get_clock();
    tf_ = config().blackboard->get<nav2::TransformBuffer::SharedPtr>("tf_buffer");
    transform_staleness_threshold_ = node->declare_or_get_parameter(
      "transform_staleness_threshold", 0.0);
  }
}

void RecoveryNode::updateRetryCount()
{
  if (!(reset_distance_ > 0.0)) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getFreshPose(
      *tf_, global_frame_, robot_frame_, clock_->now(), transform_staleness_threshold_, pose))
  {
    return;
  }

  if (!reset_pose_ || nav2_util::geometry_utils::euclidean_distance(
      reset_pose_->pose, pose.pose) >= reset_distance_)
  {
    if (reset_pose_) {
      retry_count_ = 0;
    }
    reset_pose_ = pose;
  }
}

BT::NodeStatus RecoveryNode::tick()
{
  getInput("number_of_retries", number_of_retries_);
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  if (retry_count_ > 0) {
    updateRetryCount();
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SKIPPED:
          // If first child is skipped, the entire branch is considered skipped
          halt();
          return BT::NodeStatus::SKIPPED;

        case BT::NodeStatus::SUCCESS:
          // reset node and return success when first child returns success
          // also halt the recovery action as the main action is successful, reset its state
          ControlNode::haltChild(1);
          halt();
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::FAILURE:
          {
            if (retry_count_ == 0 && !reset_pose_) {
              updateRetryCount();
            }
            if (retry_count_ < number_of_retries_) {
              // halt first child and tick second child in next iteration
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
              // reset node and return failure when max retries has been exceeded
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        default:
          throw BT::LogicError("A child node must never return IDLE");
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SKIPPED:
          {
            // if we skip the recovery (maybe a precondition fails), then we
            // should assume that no recovery is possible. For this reason,
            // we should return FAILURE and reset the index.
            // This does not count as a retry.
            current_child_idx_ = 0;
            ControlNode::haltChild(1);
            return BT::NodeStatus::FAILURE;
          }
        case BT::NodeStatus::RUNNING:
          return child_status;

        case BT::NodeStatus::SUCCESS:
          {
            // halt second child, increment recovery count, and tick first child in next iteration
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_ = 0;
          }
          break;

        case BT::NodeStatus::FAILURE:
          // reset node and return failure if second child fails
          halt();
          return BT::NodeStatus::FAILURE;

        default:
          throw BT::LogicError("A child node must never return IDLE");
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
  reset_pose_.reset();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}
