// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "nav2_ros_common/tf2_factories.hpp"

#include "nav2_behavior_tree/plugins/action/get_current_pose_action.hpp"

namespace nav2_behavior_tree
{

GetCurrentPoseAction::GetCurrentPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  tf_ = config().blackboard->get<nav2::TransformBuffer::SharedPtr>("tf_buffer");
  node->get_parameter("transform_tolerance", transform_tolerance_);
  transform_staleness_threshold_ = node->declare_or_get_parameter(
    "transform_staleness_threshold", 0.0);
  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
}

inline BT::NodeStatus GetCurrentPoseAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  geometry_msgs::msg::PoseStamped current_pose;

  geometry_msgs::msg::TransformStamped transform;
  auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  if (!nav2_util::lookupTransformWithStalenessCheck(
      *tf_, global_frame_, robot_base_frame_, node->now(), transform_staleness_threshold_,
      transform))
  {
    RCLCPP_WARN(
      config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node")->get_logger(),
      "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }
  current_pose = nav2_util::transformToPoseStamped(transform);

  setOutput("current_pose", current_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetCurrentPoseAction>("GetCurrentPose");
}
