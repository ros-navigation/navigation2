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

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/truncate_path_node.hpp"

namespace nav2_behavior_tree
{

TruncatePath::TruncatePath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  distance_(1.0)
{
  getInput("distance", distance_);
}

inline BT::NodeStatus TruncatePath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  BT::NodeStatus ret_status = child_node_->executeTick();

  nav_msgs::msg::Path input_path;
  nav_msgs::msg::Path output_path;

  getInput("input_path", input_path);

  output_path.header = input_path.header;

  geometry_msgs::msg::PoseStamped final_pose = input_path.poses.back();

  double distance_to_goal;
  do {
    distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
      input_path.poses.front(), final_pose);

    output_path.poses.push_back(input_path.poses.front());
    input_path.poses.erase(input_path.poses.begin());
  } while (distance_to_goal > distance_);


  double final_angle = atan2(
    final_pose.pose.position.y - output_path.poses.back().pose.position.y,
    final_pose.pose.position.x - output_path.poses.back().pose.position.x);

  output_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    final_angle);

  setOutput("output_path", output_path);

  return ret_status;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TruncatePath>("TruncatePath");
}
