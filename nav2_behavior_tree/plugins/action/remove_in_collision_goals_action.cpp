// Copyright (c) 2024 Angsa Robotics
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

#include "nav2_behavior_tree/plugins/action/remove_in_collision_goals_action.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_behavior_tree
{

RemoveInCollisionGoals::RemoveInCollisionGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  initialized_(false),
  costmap_cost_service_("/global_costmap/get_cost_global_costmap"),
  use_footprint_(true),
  cost_threshold_(253)
{}

void RemoveInCollisionGoals::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");

  getInput("costmap_cost_service", costmap_cost_service_);

  get_cost_client_ = node_->create_client<nav2_msgs::srv::GetCosts>(
    costmap_cost_service_);
}

inline BT::NodeStatus RemoveInCollisionGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!initialized_) {
    initialize();
  }

  getInput("use_footprint", use_footprint_);
  getInput("cost_threshold", cost_threshold_);

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  Goals valid_goal_poses;
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  request->use_footprint = use_footprint_;

  for (const auto & goal : goal_poses) {
    geometry_msgs::msg::Pose2D pose;
    pose.x = goal.pose.position.x;
    pose.y = goal.pose.position.y;
    pose.theta = tf2::getYaw(goal.pose.orientation);
    request->poses.push_back(pose);
  }

  auto future = get_cost_client_->async_send_request(request);
  auto ret = rclcpp::spin_until_future_complete(node_, future, server_timeout_);
  if (ret == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    for (size_t i = 0; i < response->costs.size(); ++i) {
      if (response->costs[i] <= cost_threshold_) {
        valid_goal_poses.push_back(goal_poses[i]);
      }
    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "RemoveInCollisionGoals BT node failed to call GetCost service of costmap");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("output_goals", valid_goal_poses);
  return BT::NodeStatus::SUCCESS;
}

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>("RemoveInCollisionGoals");
}
