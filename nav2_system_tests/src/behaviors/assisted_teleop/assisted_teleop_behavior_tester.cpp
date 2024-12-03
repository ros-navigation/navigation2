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

#include "assisted_teleop_behavior_tester.hpp"
#include "nav2_util/geometry_utils.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT

namespace nav2_system_tests
{

AssistedTeleopBehaviorTester::AssistedTeleopBehaviorTester()
: is_active_(false),
  initial_pose_received_(false)
{
  node_ = rclcpp::Node::make_shared("assisted_teleop_behavior_test");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  client_ptr_ = rclcpp_action::create_client<AssistedTeleop>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "assisted_teleop");

  initial_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  preempt_pub_ =
    node_->create_publisher<std_msgs::msg::Empty>("preempt_teleop", 10);

  cmd_vel_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_teleop", 10);

  subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AssistedTeleopBehaviorTester::amclPoseCallback, this, std::placeholders::_1));

  filtered_vel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&AssistedTeleopBehaviorTester::filteredVelCallback, this, std::placeholders::_1));

  std::string costmap_topic = "/local_costmap/costmap_raw";
  std::string footprint_topic = "/local_costmap/published_footprint";

  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node_,
    costmap_topic);

  footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
    node_,
    footprint_topic,
    *tf_buffer_);

  collision_checker_ = std::make_unique<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_,
    *footprint_sub_
  );

  stamp_ = node_->now();
}

AssistedTeleopBehaviorTester::~AssistedTeleopBehaviorTester()
{
  if (is_active_) {
    deactivate();
  }
}

void AssistedTeleopBehaviorTester::activate()
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

  // Wait for lifecycle_manager_navigation to activate behavior_server
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

  RCLCPP_INFO(this->node_->get_logger(), "Assisted Teleop action server is ready");
  is_active_ = true;
}

void AssistedTeleopBehaviorTester::deactivate()
{
  if (!is_active_) {
    throw std::runtime_error("Trying to deactivate while already inactive");
  }
  is_active_ = false;
}

bool AssistedTeleopBehaviorTester::defaultAssistedTeleopTest(
  const float lin_vel,
  const float ang_vel)
{
  if (!is_active_) {
    RCLCPP_ERROR(node_->get_logger(), "Not activated");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto goal_handle_future = client_ptr_->async_send_goal(nav2_msgs::action::AssistedTeleop::Goal());

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<AssistedTeleop>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_ptr_->async_get_result(goal_handle);

  rclcpp::Rate r(1);

  counter_ = 0;
  auto start_time = std::chrono::system_clock::now();
  while (rclcpp::ok()) {
    geometry_msgs::msg::TwistStamped cmd_vel = geometry_msgs::msg::TwistStamped();
    cmd_vel.twist.linear.x = lin_vel;
    cmd_vel.twist.angular.z = ang_vel;
    cmd_vel_pub_->publish(cmd_vel);

    if (counter_ > 1) {
      break;
    }

    auto current_time = std::chrono::system_clock::now();
    if (current_time - start_time > 25s) {
      RCLCPP_ERROR(node_->get_logger(), "Exceeded Timeout");
      return false;
    }

    rclcpp::spin_some(node_);
    r.sleep();
  }

  auto preempt_msg = std_msgs::msg::Empty();
  preempt_pub_->publish(preempt_msg);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<AssistedTeleop>::WrappedResult
    wrapped_result = result_future.get();

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

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, "odom")) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  geometry_msgs::msg::Pose2D pose_2d;
  pose_2d.x = current_pose.pose.position.x;
  pose_2d.y = current_pose.pose.position.y;
  pose_2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!collision_checker_->isCollisionFree(pose_2d)) {
    RCLCPP_ERROR(node_->get_logger(), "Ended in collision");
    return false;
  }

  return true;
}

void AssistedTeleopBehaviorTester::sendInitialPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = stamp_;
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

  initial_pose_pub_->publish(pose);
  RCLCPP_INFO(node_->get_logger(), "Sent initial pose");
}

void AssistedTeleopBehaviorTester::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
{
  initial_pose_received_ = true;
}

void AssistedTeleopBehaviorTester::filteredVelCallback(
  geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->twist.linear.x == 0.0f) {
    counter_++;
  } else {
    counter_ = 0;
  }
}

}  // namespace nav2_system_tests
