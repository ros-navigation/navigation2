// Copyright (c) 2018 Intel Corporation
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

#include "nav2_simple_navigator/simple_navigator.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>

using namespace std::chrono_literals;

namespace nav2_simple_navigator
{

SimpleNavigator::SimpleNavigator()
: nav2_lifecycle::LifecycleNode("simple_navigator", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("simple_navigator_client_node");

  self_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "NavigateToPose");

  goal_sub_ = rclcpp_node_->create_subscription<geometry_msgs::msg::PoseStamped>("goal",
      std::bind(&SimpleNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  // Create our two task clients
  planner_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(client_node_,
      "ComputePathToPose");

  controller_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(client_node_,
      "FollowPath");

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "NavigateToPose",
      std::bind(&SimpleNavigator::navigateToPose, this, std::placeholders::_1));

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  planner_client_.reset();
  controller_client_.reset();
  action_server_.reset();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_ERROR(get_logger(), "Handling error state");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

void
SimpleNavigator::navigateToPose(const std::shared_ptr<GoalHandle> goal_handle)
{
  // Initialize the overall NavigateToPose goal and result
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Set up the input and output for the global planner
  nav2_msgs::action::ComputePathToPose::Goal planner_goal;
  planner_goal.pose = goal->pose;

  RCLCPP_DEBUG(get_logger(), "Getting the path from the planner for goal pose:");
  RCLCPP_DEBUG(get_logger(), "position.x: %f", goal->pose.pose.position.x);
  RCLCPP_DEBUG(get_logger(), "position.y: %f", goal->pose.pose.position.y);
  RCLCPP_DEBUG(get_logger(), "position.z: %f", goal->pose.pose.position.z);
  RCLCPP_DEBUG(get_logger(), "orientation.x: %f", goal->pose.pose.orientation.x);
  RCLCPP_DEBUG(get_logger(), "orientation.y: %f", goal->pose.pose.orientation.y);
  RCLCPP_DEBUG(get_logger(), "orientation.z: %f", goal->pose.pose.orientation.z);
  RCLCPP_DEBUG(get_logger(), "orientation.w: %f", goal->pose.pose.orientation.w);

  planner_client_->wait_for_action_server();

  // Send the goal pose to the planner
  auto planner_future_goal_handle = planner_client_->async_send_goal(planner_goal);
  if (rclcpp::spin_until_future_complete(client_node_, planner_future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Send goal call failed");
    goal_handle->abort(result);
    return;
  }

  // Get the goal handle
  auto planner_goal_handle = planner_future_goal_handle.get();
  if (!planner_goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    goal_handle->abort(result);
    return;
  }

  // Wait for the action to complete
  auto planner_future_result = planner_goal_handle->async_result();
  if (rclcpp::spin_until_future_complete(client_node_, planner_future_result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Get result call failed");
    goal_handle->abort(result);
    return;
  }

  // Get the final result
  auto planner_result = planner_future_result.get();
  if (planner_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(get_logger(), "Get wrapped result call failed");
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Received path of size %u from planner",
    planner_result.result->path.poses.size());

  // Print the path received from the planner
  int index = 0;
  for (auto pose : planner_result.result->path.poses) {
    RCLCPP_DEBUG(get_logger(), "Point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  controller_client_->wait_for_action_server();

  // Set up the input for the local planner
  nav2_msgs::action::FollowPath::Goal controller_goal;
  controller_goal.path = planner_result.result->path;

  // Send the goal pose to the local planner
  RCLCPP_INFO(get_logger(), "Sending path to the controller to execute");
  auto controller_future_goal_handle = controller_client_->async_send_goal(controller_goal);
  if (rclcpp::spin_until_future_complete(client_node_, controller_future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Send goal call failed");
    goal_handle->abort(result);
    return;
  }

  // Get the goal handle
  auto controller_goal_handle = controller_future_goal_handle.get();
  if (!controller_goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    goal_handle->abort(result);
    return;
  }

  auto controller_future_result = controller_goal_handle->async_result();
  rclcpp::executor::FutureReturnCode rc;
  do {
    rc = rclcpp::spin_until_future_complete(client_node_, controller_future_result, 10ms);

    if (goal_handle->is_canceling()) {
      auto controller_future_cancel_result = controller_client_->async_cancel_goal(
        controller_goal_handle);
      if (rclcpp::spin_until_future_complete(client_node_, controller_future_cancel_result) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(client_node_->get_logger(), "failed to cancel controller goal");
      }

      goal_handle->canceled(result);
      return;
    }
  } while (rc == rclcpp::executor::FutureReturnCode::TIMEOUT);

  // The return for the overall navigation is the result of the local planner
  auto controller_result = controller_future_result.get();
  switch (controller_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_handle->succeed(result);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      goal_handle->abort(result);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      goal_handle->canceled(result);
      break;

    default:
      throw std::logic_error("BtActionNode::Tick: invalid status value");
  }
}

void
SimpleNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_simple_navigator
