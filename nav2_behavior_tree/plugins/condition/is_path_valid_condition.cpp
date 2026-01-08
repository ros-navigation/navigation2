// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

namespace nav2_behavior_tree
{

IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  max_cost_(254), consider_unknown_as_obstacle_(false), layer_name_(""), footprint_(""),
  check_full_path_(false)
{
  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  client_ =
    node_->create_client<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    false /* Does not create and spin an internal executor*/);
}

void IsPathValidCondition::initialize()
{
  getInputOrBlackboard("server_timeout", server_timeout_);
  getInput<unsigned int>("max_cost", max_cost_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  getInput<std::string>("layer_name", layer_name_);
  getInput<std::string>("footprint", footprint_);
  getInput<bool>("check_full_path", check_full_path_);
}

BT::NodeStatus IsPathValidCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  request->max_cost = max_cost_;
  request->consider_unknown_as_obstacle = consider_unknown_as_obstacle_;
  request->layer_name = layer_name_;
  request->footprint = footprint_;
  request->check_full_path = check_full_path_;
  auto response = client_->invoke(request, server_timeout_);

  // Check if validation was successful
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "IsPathValid service failed to validate path");
    return BT::NodeStatus::FAILURE;
  }

  if (response->is_valid) {
    // Ensure collision_poses output is cleared when the path is valid
    std::vector<geometry_msgs::msg::PoseStamped> collision_poses;
    setOutput("collision_poses", collision_poses);
    return BT::NodeStatus::SUCCESS;
  }

  // Extract collision poses based on invalid pose indices
  std::vector<geometry_msgs::msg::PoseStamped> collision_poses;
  if (!response->invalid_pose_indices.empty()) {
    std::stringstream ss;
    ss << "Path validation failed. Invalid pose indices: [";
    for (size_t i = 0; i < response->invalid_pose_indices.size(); ++i) {
      int32_t idx = response->invalid_pose_indices[i];
      ss << idx;
      if (i < response->invalid_pose_indices.size() - 1) {
        ss << ", ";
      }
      // Add the collision pose if index is valid
      if (idx >= 0 && static_cast<size_t>(idx) < path.poses.size()) {
        collision_poses.push_back(path.poses[idx]);
      }
    }
    ss << "]";
    RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
  }

  // Set collision poses output
  setOutput("collision_poses", collision_poses);

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
}
