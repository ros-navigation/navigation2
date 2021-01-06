// Copyright (c) 2020 Samsung Research
// Copyright (c) 2020 Sarthak Mittal
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
// limitations under the License. Reserved.

#include <string>
#include <random>
#include <tuple>
#include <memory>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "wait_recovery_tester.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT

namespace nav2_system_tests
{

WaitRecoveryTester::WaitRecoveryTester()
: is_active_(false),
  initial_pose_received_(false)
{
  node_ = rclcpp::Node::make_shared("wait_recovery_test");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  client_ptr_ = rclcpp_action::create_client<Wait>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "wait");

  publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&WaitRecoveryTester::amclPoseCallback, this, std::placeholders::_1));
}

WaitRecoveryTester::~WaitRecoveryTester()
{
  if (is_active_) {
    deactivate();
  }
}

void WaitRecoveryTester::activate()
{
  if (is_active_) {
    throw std::runtime_error("Trying to activate while already active");
    return;
  }

  while (!initial_pose_received_) {
    RCLCPP_WARN(node_->get_logger(), "Initial pose not received");
    sendInitialPose();
    std::this_thread::sleep_for(100ms);
    rclcpp::spin_some(node_);
  }

  // Wait for lifecycle_manager_navigation to activate recoveries_server
  std::this_thread::sleep_for(10s);

  if (!client_ptr_) {
    RCLCPP_ERROR(node_->get_logger(), "Action client not initialized");
    is_active_ = false;
    return;
  }

  if (!client_ptr_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    is_active_ = false;
    return;
  }

  RCLCPP_INFO(this->node_->get_logger(), "Wait action server is ready");
  is_active_ = true;
}

void WaitRecoveryTester::deactivate()
{
  if (!is_active_) {
    throw std::runtime_error("Trying to deactivate while already inactive");
  }
  is_active_ = false;
}

bool WaitRecoveryTester::recoveryTest(
  const float wait_time)
{
  if (!is_active_) {
    RCLCPP_ERROR(node_->get_logger(), "Not activated");
    return false;
  }

  // Sleep to let recovery server be ready for serving in multiple runs
  std::this_thread::sleep_for(5s);

  auto start_time = node_->now();
  auto goal_msg = Wait::Goal();
  goal_msg.time = rclcpp::Duration(wait_time, 0.0);

  RCLCPP_INFO(this->node_->get_logger(), "Sending goal");

  auto goal_handle_future = client_ptr_->async_send_goal(goal_msg);

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<Wait>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_ptr_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<Wait>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: break;
    case rclcpp_action::ResultCode::ABORTED: RCLCPP_ERROR(
        node_->get_logger(),
        "Goal was aborted");
      return false;
    case rclcpp_action::ResultCode::CANCELED: RCLCPP_ERROR(
        node_->get_logger(),
        "Goal was canceled");
      return false;
    default: RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return false;
  }

  RCLCPP_INFO(node_->get_logger(), "result received");


  if ((node_->now() - start_time).seconds() < static_cast<double>(wait_time)) {
    return false;
  }

  return true;
}

bool WaitRecoveryTester::recoveryTestCancel(
  const float wait_time)
{
  if (!is_active_) {
    RCLCPP_ERROR(node_->get_logger(), "Not activated");
    return false;
  }

  // Sleep to let recovery server be ready for serving in multiple runs
  std::this_thread::sleep_for(5s);

  auto start_time = node_->now();
  auto goal_msg = Wait::Goal();
  goal_msg.time = rclcpp::Duration(wait_time, 0.0);

  RCLCPP_INFO(this->node_->get_logger(), "Sending goal");

  auto goal_handle_future = client_ptr_->async_send_goal(goal_msg);

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<Wait>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_ptr_->async_cancel_all_goals();

  RCLCPP_INFO(node_->get_logger(), "Waiting for cancellation");
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get cancel result call failed :(");
    return false;
  }

  auto status = goal_handle_future.get()->get_status();

  switch (status) {
    case rclcpp_action::GoalStatus::STATUS_SUCCEEDED: RCLCPP_ERROR(
        node_->get_logger(),
        "Goal succeeded");
      return false;
    case rclcpp_action::GoalStatus::STATUS_ABORTED: RCLCPP_ERROR(
        node_->get_logger(),
        "Goal was aborted");
      return false;
    case rclcpp_action::GoalStatus::STATUS_CANCELED: RCLCPP_INFO(
        node_->get_logger(),
        "Goal was canceled");
      return true;
    case rclcpp_action::GoalStatus::STATUS_CANCELING: RCLCPP_INFO(
        node_->get_logger(),
        "Goal is cancelling");
      return true;
    case rclcpp_action::GoalStatus::STATUS_EXECUTING: RCLCPP_ERROR(
        node_->get_logger(),
        "Goal is executing");
      return false;
    case rclcpp_action::GoalStatus::STATUS_ACCEPTED: RCLCPP_ERROR(
        node_->get_logger(),
        "Goal is processing");
      return false;
    default: RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return false;
  }

  return false;
}

void WaitRecoveryTester::sendInitialPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = rclcpp::Time();
  pose.pose.pose.position.x = -2.0;
  pose.pose.pose.position.y = -0.5;
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation.x = 0.0;
  pose.pose.pose.orientation.y = 0.0;
  pose.pose.pose.orientation.z = 0.0;
  pose.pose.pose.orientation.w = 1.0;
  for (int i = 0; i < 35; i++) {
    pose.pose.covariance[i] = 0.0;
  }
  pose.pose.covariance[0] = 0.08;
  pose.pose.covariance[7] = 0.08;
  pose.pose.covariance[35] = 0.05;

  publisher_->publish(pose);
  RCLCPP_INFO(node_->get_logger(), "Sent initial pose");
}

void WaitRecoveryTester::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
{
  initial_pose_received_ = true;
}

}  // namespace nav2_system_tests
