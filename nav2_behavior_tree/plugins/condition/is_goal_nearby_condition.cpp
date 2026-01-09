// Copyright (c) 2026 Jakub Chudziński
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

#include "nav2_behavior_tree/plugins/condition/is_goal_nearby_condition.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

IsGoalNearbyCondition::IsGoalNearbyCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  transform_tolerance_(0.1)
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
}

BT::NodeStatus IsGoalNearbyCondition::tick()
{
  nav_msgs::msg::Path new_path;
  double prox_thr = 0.0;
  double max_robot_pose_search_dist = -1.0;
  getInput("path", new_path);
  getInput("proximity_threshold", prox_thr);
  getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

  if(new_path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Path is empty");
    return BT::NodeStatus::FAILURE;
  }

  bool path_pruning = true;
  if (max_robot_pose_search_dist < 0.0) {
    path_pruning = false;
  }


  if (!path_pruning || new_path != path_) {
    path_ = new_path;
    closest_pose_detection_begin_ = path_.poses.begin();
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot pose");
    return BT::NodeStatus::FAILURE;
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::transformPoseInTargetFrame(pose, robot_pose, *tf_buffer_,
    path_.header.frame_id))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to transform robot pose to path frame '%s'",
      path_.header.frame_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto closest_pose_upper_bound = path_.poses.end();
  if (path_pruning) {
    closest_pose_upper_bound =
      nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
  }

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto closest_pose_it =
    nav2_util::geometry_utils::min_by(
    closest_pose_detection_begin_, closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps)
    {
      return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
    });

  closest_pose_detection_begin_ = closest_pose_it;

  const std::size_t closest_index =
    static_cast<std::size_t>(closest_pose_it - path_.poses.begin());
  const double remaining_length =
    nav2_util::geometry_utils::calculate_path_length(path_, closest_index);

  return (remaining_length < prox_thr) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsGoalNearbyCondition>("IsGoalNearby");
}
