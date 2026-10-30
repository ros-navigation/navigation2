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

#include "nav2_ros_common/tf_manager.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/distance_traveled_condition.hpp"

namespace nav2_behavior_tree
{

DistanceTraveledCondition::DistanceTraveledCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  distance_(1.0)
{
}

void DistanceTraveledCondition::initialize()
{
  getInput("distance", distance_);

  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  clock_ = node_->get_clock();
  tf_manager_ = config().blackboard->get<nav2::TransformManager::SharedPtr>("tf_buffer");
  transform_staleness_threshold_ = node_->declare_or_get_parameter(
    "transform_staleness_threshold", 0.0);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
}

BT::NodeStatus DistanceTraveledCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
    const auto start_pose_result = tf_manager_->getLatestPose(
      global_frame_, robot_base_frame_, transform_staleness_threshold_, nav2::Log::Disabled);
    if (!start_pose_result) {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }
    start_pose_ = *start_pose_result;
    return BT::NodeStatus::FAILURE;
  }

  // Determine distance travelled since we've started this iteration

  const auto current_pose = tf_manager_->getLatestPose(
    global_frame_, robot_base_frame_, transform_staleness_threshold_, nav2::Log::Disabled);
  if (!current_pose) {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Get euclidean distance
  auto travelled = nav2_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose->pose);

  if (travelled < distance_) {
    return BT::NodeStatus::FAILURE;
  }

  // Update start pose
  start_pose_ = *current_pose;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceTraveledCondition>("DistanceTraveled");
}
