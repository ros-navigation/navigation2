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
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/concatenate_paths_action.hpp"

namespace nav2_behavior_tree
{

ConcatenatePaths::ConcatenatePaths(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

void ConcatenatePaths::initialize()
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  tf_ = config().blackboard->get<nav2::TransformBuffer::SharedPtr>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus ConcatenatePaths::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path1, input_path2;
  getInput("input_path1", input_path1);
  getInput("input_path2", input_path2);

  if (input_path1.poses.empty() && input_path2.poses.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No input paths provided to concatenate. Both paths are empty.");
    return BT::NodeStatus::FAILURE;
  }

  const std::string & frame1 = input_path1.header.frame_id;
  const std::string & frame2 = input_path2.header.frame_id;
  if (!input_path2.poses.empty() && frame1 != frame2) {
    bool transform_to_path1_frame = false;
    getInput("transform_to_path1_frame", transform_to_path1_frame);
    if (!transform_to_path1_frame) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cannot concatenate paths in different frames ('%s' and '%s'). "
        "Set transform_to_path1_frame to transform the second path into the frame of the first.",
        frame1.c_str(), frame2.c_str());
      return BT::NodeStatus::FAILURE;
    }

    nav_msgs::msg::Path transformed_path2;
    if (!nav2_util::transformPathInTargetFrame(
        input_path2, transformed_path2, *tf_, frame1, transform_tolerance_))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to transform the second path from '%s' to '%s' for concatenation.",
        frame2.c_str(), frame1.c_str());
      return BT::NodeStatus::FAILURE;
    }
    input_path2 = transformed_path2;
  }

  nav_msgs::msg::Path output_path;
  output_path = input_path1;
  if (input_path1.header != std_msgs::msg::Header()) {
    output_path.header = input_path1.header;
  } else if (input_path2.header != std_msgs::msg::Header()) {
    output_path.header = input_path2.header;
  }

  output_path.poses.insert(
    output_path.poses.end(),
    input_path2.poses.begin(),
    input_path2.poses.end());

  setOutput("output_path", output_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ConcatenatePaths>("ConcatenatePaths");
}
