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
#include <string>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_lifecycle_manager
{
using nav2_util::geometry_utils::orientationAroundZAxis;

LifecycleManagerClient::LifecycleManagerClient(const std::string & name)
{
  manage_service_name_ = name + std::string("/manage_nodes");
  active_service_name_ = name + std::string("/is_active");

  // Create the node to use for all of the service clients
  node_ = std::make_shared<rclcpp::Node>(name + "_service_client");

  // Create the service clients
  manager_client_ = node_->create_client<ManageLifecycleNodes>(manage_service_name_);
  is_active_client_ = node_->create_client<std_srvs::srv::Trigger>(active_service_name_);

  navigate_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

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

SystemStatus
LifecycleManagerClient::is_active(const std::chrono::nanoseconds timeout)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  RCLCPP_INFO(
    node_->get_logger(), "Waiting for the %s service...",
    active_service_name_.c_str());

  if (!is_active_client_->wait_for_service(timeout)) {
    return SystemStatus::TIMEOUT;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Sending %s request",
    active_service_name_.c_str());
  auto future_result = is_active_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, timeout) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return SystemStatus::TIMEOUT;
  }

  if (future_result.get()->success) {
    return SystemStatus::ACTIVE;
  } else {
    return SystemStatus::INACTIVE;
  }
}

void
LifecycleManagerClient::set_initial_pose(double x, double y, double theta)
{
  const double PI = 3.141592653589793238463;
  auto pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

  pose->header.frame_id = "map";
  pose->header.stamp = node_->now();
  pose->pose.pose.position.x = x;
  pose->pose.pose.position.y = y;
  pose->pose.pose.position.z = 0.0;
  pose->pose.pose.orientation = orientationAroundZAxis(theta);
  pose->pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  pose->pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  pose->pose.covariance[6 * 5 + 5] = PI / 12.0 * PI / 12.0;

  initial_pose_publisher_->publish(std::move(pose));
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

  // Send it
  auto future_goal_handle = navigate_action_client_->async_send_goal(goal);
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
  auto future_result = navigate_action_client_->async_get_result(goal_handle);

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

  RCLCPP_INFO(
    node_->get_logger(), "Waiting for the %s service...",
    manage_service_name_.c_str());

  while (!manager_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Client interrupted while waiting for service to appear");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_INFO(
    node_->get_logger(), "Sending %s request",
    manage_service_name_.c_str());
  auto future_result = manager_client_->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, future_result);
  return future_result.get()->success;
}

}  // namespace nav2_lifecycle_manager
