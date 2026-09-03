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
#include <limits>
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0),
  transform_tolerance_(0.1),
  reset_distance_(std::numeric_limits<double>::infinity()),
  reset_time_(std::numeric_limits<double>::infinity()),
  first_tick_(true)
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  tf_ = config().blackboard->get<nav2::TransformBuffer::SharedPtr>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
}

void RecoveryNode::checkAndResetCounterIfNeeded()
{
  if (!std::isinf(reset_distance_) || !std::isinf(reset_time_)) {
    geometry_msgs::msg::PoseStamped current_pose;
    const bool has_pose = nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_);

    const rclcpp::Time current_time = node_->now();

    if (first_tick_) {
      if (has_pose) {
        last_reset_pose_ = current_pose;
      }
      last_reset_time_ = current_time;
      first_tick_ = false;
      return;
    }

    bool reset_needed = false;

    // Check distance reset
    if (!std::isinf(reset_distance_) && has_pose && last_reset_pose_.header.frame_id != "") {
      double travelled = nav2_util::geometry_utils::euclidean_distance(
        last_reset_pose_.pose, current_pose.pose);
      if (travelled >= reset_distance_) {
        reset_needed = true;
      }
    }

    // Check time reset
    if (!std::isinf(reset_time_) && last_reset_time_.nanoseconds() > 0) {
      double elapsed = (current_time - last_reset_time_).seconds();
      if (elapsed >= reset_time_) {
        reset_needed = true;
      }
    }

    if (reset_needed) {
      retry_count_ = 0;
      if (has_pose) {
        last_reset_pose_ = current_pose;
      }
      last_reset_time_ = current_time;
    }
  }
}

BT::NodeStatus RecoveryNode::tick()
{
  getInput("number_of_retries", number_of_retries_);
  getInput("reset_distance", reset_distance_);
  getInput("reset_time", reset_time_);

  checkAndResetCounterIfNeeded();

  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
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
  first_tick_ = true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}
