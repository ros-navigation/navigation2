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
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_behavior_tree
{

RemoveInCollisionGoals::RemoveInCollisionGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  initialized_(false),
  which_costmap_("both"),
  cost_threshold_(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
{}

void RemoveInCollisionGoals::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  get_global_cost_client_ = node_->create_client<nav2_msgs::srv::GetCost>(
    "global_costmap/get_cost_global_costmap");
  get_local_cost_client_ = node_->create_client<nav2_msgs::srv::GetCost>(
    "local_costmap/get_cost_local_costmap");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
}

inline BT::NodeStatus RemoveInCollisionGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!initialized_) {
    initialize();
  }

  getInput("costmap", which_costmap_);
  getInput("cost_threshold", cost_threshold_);

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  Goals valid_goal_poses;
  for (const auto & goal : goal_poses) {
    auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
    request->x = goal.pose.position.x;
    request->y = goal.pose.position.y;
    request->theta = tf2::getYaw(goal.pose.orientation);
    request->use_footprint = true;

    if (which_costmap_ == "global") {
      auto future_global = get_global_cost_client_->async_send_request(request);
      auto ret_global = rclcpp::spin_until_future_complete(node_, future_global, server_timeout_);
      if (ret_global == rclcpp::FutureReturnCode::SUCCESS) {
        if (future_global.get()->cost <= cost_threshold_) {
          valid_goal_poses.push_back(goal);
        }
      } else {
        RCLCPP_ERROR(
          node_->get_logger(),
          "RemoveInCollisionGoals BT node failed to call GetCost service of global costmap");
        return BT::NodeStatus::FAILURE;
      }
    } else if (which_costmap_ == "local") {
      auto future_local = get_local_cost_client_->async_send_request(request);
      auto ret_local = rclcpp::spin_until_future_complete(node_, future_local, server_timeout_);
      if (ret_local == rclcpp::FutureReturnCode::SUCCESS) {
        if (future_local.get()->cost <= cost_threshold_) {
          valid_goal_poses.push_back(goal);
        }
      } else {
        RCLCPP_ERROR(
          node_->get_logger(),
          "RemoveInCollisionGoals BT node failed to call GetCost service of local costmap");
        return BT::NodeStatus::FAILURE;
      }
    } else if (which_costmap_ == "both") {
      auto future_global = get_global_cost_client_->async_send_request(request);
      auto future_local = get_local_cost_client_->async_send_request(request);
      auto ret_global = rclcpp::spin_until_future_complete(node_, future_global, server_timeout_);
      auto ret_local = rclcpp::spin_until_future_complete(node_, future_local, server_timeout_);
      if (ret_local == rclcpp::FutureReturnCode::SUCCESS &&
        ret_global == rclcpp::FutureReturnCode::SUCCESS)
      {
        if (future_local.get()->cost <= cost_threshold_ &&
          future_global.get()->cost <= cost_threshold_)
        {
          valid_goal_poses.push_back(goal);
        }
      } else {
        RCLCPP_ERROR(
          node_->get_logger(),
          "RemoveInCollisionGoals BT node failed to call GetCost service of local or global costmap");
        return BT::NodeStatus::FAILURE;
      }
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "The costmap parameter must be either 'local', 'global', or 'both'");
      return BT::NodeStatus::FAILURE;
    }
  }
  setOutput("output_goals", valid_goal_poses);
  return BT::NodeStatus::SUCCESS;
}

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>("RemoveInCollisionGoals");
}
