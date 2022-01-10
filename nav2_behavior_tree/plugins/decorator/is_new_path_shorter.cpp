// Copyright (c) 2022 Pradheep Padmanabhan - Neobotix GmbH
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
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/is_new_path_shorter.hpp"

namespace nav2_behavior_tree
{

IsNewPathShorter::IsNewPathShorter(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::RetryNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Initializing the paths
  new_path.poses.resize(1);
  new_path.poses[0].pose.position.x = 0.0;
  old_path.poses.resize(1);
  old_path.poses[0].pose.position.x = 0.0;
}

inline BT::NodeStatus IsNewPathShorter::tick()
{
  getInput("path", new_path);
  getInput("allowed_goal_distance_proximity", allowed_goal_distance_proximity);

  // Check if the old path is not same with the current one
  if (new_path != old_path && old_path.poses.size() != 0 && new_path.poses.size() != 0) {
    // Calculate and compare the old and the new path length, given the goal proximity
    if ( (nav2_util::geometry_utils::calculate_path_length(new_path, 0) -
      nav2_util::geometry_utils::calculate_path_length(
        old_path,
        0) >= allowed_goal_distance_proximity
      ) && (old_path.poses.back() == new_path.poses.back()) && !first_time_)
    {
      const BT::NodeStatus child_state = child_node_->executeTick();
      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::SUCCESS:
          first_time_ = true;
          return BT::NodeStatus::SUCCESS;
        case BT::NodeStatus::FAILURE:
          return BT::NodeStatus::FAILURE;
        default:
          return BT::NodeStatus::FAILURE;
      }
    }
  }
  first_time_ = false;
  old_path = new_path;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsNewPathShorter>("IsNewPathShorter");
}
