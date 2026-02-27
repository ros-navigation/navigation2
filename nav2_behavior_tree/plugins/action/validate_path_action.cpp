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

#include <string>
#include <memory>
#include <limits>

#include "nav2_behavior_tree/plugins/action/validate_path_action.hpp"

namespace nav2_behavior_tree
{

ValidatePath::ValidatePath(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::IsPathValid>(service_node_name, conf, "is_path_valid")
{
}


void ValidatePath::on_tick()
{
  getInput<unsigned int>("max_cost", max_cost_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  getInput<std::string>("layer_name", layer_name_);
  getInput<std::string>("footprint", footprint_);
  getInput<bool>("check_full_path", check_full_path_);
  getInput("path", path_);

  request_ = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request_->path = path_;
  request_->max_cost = max_cost_;
  request_->consider_unknown_as_obstacle = consider_unknown_as_obstacle_;
  request_->layer_name = layer_name_;
  request_->footprint = footprint_;
  request_->check_full_path = check_full_path_;
}

BT::NodeStatus ValidatePath::on_completion(
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
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
      if (idx >= 0 && static_cast<size_t>(idx) < path_.poses.size()) {
        collision_poses.push_back(path_.poses[idx]);
      }
    }
    ss << "]";
    RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
  }

  // Set collision poses output
  setOutput("collision_poses", collision_poses);

  return BT::NodeStatus::FAILURE;
}

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ValidatePath>("ValidatePath");
}
