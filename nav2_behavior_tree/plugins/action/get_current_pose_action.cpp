// Copyright (c) 2025 Pau Reverté
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

#include "nav2_behavior_tree/plugins/action/get_current_pose_action.hpp"

namespace nav2_behavior_tree
{

GetCurrentPose::GetCurrentPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf),
  node_(nullptr),
  tf_(nullptr)
{
  if (!config().blackboard->get("node", node_)) {
    throw BT::RuntimeError("Node not found in blackboard.");
  }

  if (!config().blackboard->get("tf_buffer", tf_)) {
    throw BT::RuntimeError("tf_buffer not found in blackboard.");
  }

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
}

BT::NodeStatus GetCurrentPose::tick()
{
  if (!node_ || !tf_) {
    std::cerr << "[GetCurrentPose] Missing 'node' or 'tf_buffer' in Blackboard." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = global_frame_;
  current_pose.header.stamp = node_->now();

  try {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg = tf_->lookupTransform(
      global_frame_,
      robot_base_frame_,
      tf2::TimePointZero);

    current_pose.pose.position.x = tf_msg.transform.translation.x;
    current_pose.pose.position.y = tf_msg.transform.translation.y;
    current_pose.pose.position.z = tf_msg.transform.translation.z;
    current_pose.pose.orientation = tf_msg.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "[GetCurrentPose] TF Error: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("current_pose", current_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetCurrentPose>("GetCurrentPose");
}
