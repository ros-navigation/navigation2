// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/distance_traveled_condition.hpp"

namespace nav2_behavior_tree
{

DistanceTraveledCondition::DistanceTraveledCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  distance_(1.0),
  transform_tolerance_(0.1),
  is_global_(false),
  current_run_id_("")
{
}

void DistanceTraveledCondition::initialize()
{
  getInput("distance", distance_);

  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
  is_global_ = node_->declare_or_get_parameter("is_global", false);
}

BT::NodeStatus DistanceTraveledCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
    if (!is_global_) {
      if (!nav2_util::getCurrentPose(
          start_pose_, *tf_, global_frame_, robot_base_frame_,
          transform_tolerance_))
      {
        RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      }
      return BT::NodeStatus::FAILURE;
    } else if (start_pose_.header.frame_id.empty()) {
      if (!nav2_util::getCurrentPose(
          start_pose_, *tf_, global_frame_, robot_base_frame_,
          transform_tolerance_))
      {
        RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      }
    }
  }

  // Global mode: on RunID change fall through without resetting start_pose_
  if (is_global_) {
    std::string new_run_id;
    try {
      new_run_id = config().blackboard->template get<std::string>("run_id");
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "is_global=true requires 'run_id' to be set on the blackboard for condition: " +
        std::string(name()));
    }
    if (new_run_id != current_run_id_) {
      current_run_id_ = new_run_id;
      // Do not reset start_pose_ to preserve distance measurement across runs
    }
  }

  // Determine distance travelled since we've started this iteration
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Get euclidean distance
  auto travelled = nav2_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose.pose);

  if (travelled < distance_) {
    return BT::NodeStatus::FAILURE;
  }

  // Update start pose
  start_pose_ = current_pose;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceTraveledCondition>("DistanceTraveled");
}
