// Copyright (c) 2025 Open Navigation LLC
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/are_poses_near_condition.hpp"

namespace nav2_behavior_tree
{

ArePosesNearCondition::ArePosesNearCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "global_frame", this);
}

void ArePosesNearCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus ArePosesNearCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (arePosesNearby()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

bool ArePosesNearCondition::arePosesNearby()
{
  geometry_msgs::msg::PoseStamped pose1, pose2;
  double tol;
  getInput("ref_pose", pose1);
  getInput("target_pose", pose2);
  getInput("tolerance", tol);

  if (pose1.header.frame_id != pose2.header.frame_id) {
    if (!nav2_util::transformPoseInTargetFrame(
      pose1, pose1, *tf_, global_frame_, transform_tolerance_) ||
      !nav2_util::transformPoseInTargetFrame(
      pose2, pose2, *tf_, global_frame_, transform_tolerance_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to transform poses to the same frame");
      return false;
    }
  }

  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  return (dx * dx + dy * dy) <= (tol * tol);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ArePosesNearCondition>("ArePosesNear");
}
