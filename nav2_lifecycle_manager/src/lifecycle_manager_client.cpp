// Copyright (c) 2019 Intel Corporation
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

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

#include <cmath>
#include <memory>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_lifecycle_manager
{
using nav2_util::geometry_utils::orientationAroundZAxis;

LifecycleManagerClient::LifecycleManagerClient()
{
  // Create the node to use for all of the service clients
  node_ = std::make_shared<rclcpp::Node>("lifecycle_manager_client_service_client");

  // Create the service clients
  manager_client_ = node_->create_client<ManageLifecycleNodes>(service_name_);

  navigate_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "NavigateToPose");

  initial_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS());
}

bool
LifecycleManagerClient::startup()
{
  return callService(ManageLifecycleNodes::Request::STARTUP);
}

bool
LifecycleManagerClient::shutdown()
{
  return callService(ManageLifecycleNodes::Request::SHUTDOWN);
}

bool
LifecycleManagerClient::pause()
{
  return callService(ManageLifecycleNodes::Request::PAUSE);
}

bool
LifecycleManagerClient::resume()
{
  return callService(ManageLifecycleNodes::Request::RESUME);
}

bool
LifecycleManagerClient::reset()
{
  return callService(ManageLifecycleNodes::Request::RESET);
}

void
LifecycleManagerClient::set_initial_pose(double x, double y, double theta)
{
  const double PI = 3.141592653589793238463;
  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation = orientationAroundZAxis(theta);
  pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  pose.pose.covariance[6 * 5 + 5] = PI / 12.0 * PI / 12.0;

  initial_pose_publisher_->publish(pose);
}

bool
LifecycleManagerClient::navigate_to_pose(double x, double y, double theta)
{
  navigate_action_client_->wait_for_action_server();

  // Initialize the goal
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = 0;
  target_pose.pose.orientation = orientationAroundZAxis(theta);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = target_pose;

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    typename rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};

  // Send it
  auto future_goal_handle = navigate_action_client_->async_send_goal(goal, send_goal_options);
  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
    return false;
  }

  // Get the goal handle
  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  // Wait for the action to complete
  auto future_result = goal_handle->async_result();
  if (rclcpp::spin_until_future_complete(node_, future_result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed");
    return false;
  }

  // Get the final result
  auto wrapped_result = future_result.get();
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

bool
LifecycleManagerClient::callService(uint8_t command)
{
  auto request = std::make_shared<ManageLifecycleNodes::Request>();
  request->command = command;

  RCLCPP_INFO(node_->get_logger(), "Waiting for the lifecycle_manager's %s service...",
    service_name_);

  while (!manager_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Client interrupted while waiting for service to appear");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_INFO(node_->get_logger(), "send_async_request (%s) to the lifecycle_manager",
    service_name_);
  auto future_result = manager_client_->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, future_result);
  return future_result.get()->success;
}

}  // namespace nav2_lifecycle_manager
