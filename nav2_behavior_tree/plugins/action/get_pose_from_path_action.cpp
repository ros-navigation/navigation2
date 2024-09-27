// Copyright (c) 2024 Marc Morcos
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
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/get_pose_from_path_action.hpp"

namespace nav2_behavior_tree
{

GetPoseFromPath::GetPoseFromPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus GetPoseFromPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path;
  getInput("path", input_path);

  int pose_index;
  getInput("index", pose_index);

  if (input_path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // Account for negative indices
  if(pose_index < 0) {
    pose_index = input_path.poses.size() + pose_index;
  }

  // out of bounds index
  if(pose_index < 0 || static_cast<unsigned>(pose_index) >= input_path.poses.size()) {
    return BT::NodeStatus::FAILURE;
  }

  // extract pose
  geometry_msgs::msg::PoseStamped output_pose;
  output_pose = input_path.poses[pose_index];

  // populate pose frame from path if necessary
  if(output_pose.header.frame_id.empty()) {
    output_pose.header.frame_id = input_path.header.frame_id;
  }


  setOutput("pose", output_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GetPoseFromPath>("GetPoseFromPath");
}
