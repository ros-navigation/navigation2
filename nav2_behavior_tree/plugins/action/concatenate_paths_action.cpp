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

inline BT::NodeStatus ConcatenatePaths::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path1, input_path2;
  getInput("input_path1", input_path1);
  getInput("input_path2", input_path2);

  if (input_path1.poses.empty() && input_path2.poses.empty()) {
    RCLCPP_ERROR(
      config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node")->get_logger(),
      "No input paths provided to concatenate. Both paths are empty.");
    return BT::NodeStatus::FAILURE;
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
