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

#include "spin_behavior_tester.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT

namespace nav2_system_tests
{

SpinBehaviorTester::SpinBehaviorTester()
: is_active_(false),
  initial_pose_received_(false)
{
  node_ = rclcpp::Node::make_shared("spin_behavior_test");


  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  if (std::getenv("MAKE_FAKE_COSTMAP") != NULL) {
    // if this variable is set, make a fake costmap
    make_fake_costmap_ = true;
  } else {
    make_fake_costmap_ = false;
  }

  client_ptr_ = rclcpp_action::create_client<Spin>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "spin");

  publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  fake_costmap_publisher_ =
    node_->create_publisher<nav2_msgs::msg::Costmap>(
    "local_costmap/costmap_raw",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  fake_footprint_publisher_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "local_costmap/published_footprint", rclcpp::SystemDefaultsQoS());

  subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpinBehaviorTester::amclPoseCallback, this, std::placeholders::_1));

  stamp_ = node_->now();
}

SpinBehaviorTester::~SpinBehaviorTester()
{
  if (is_active_) {
    deactivate();
  }
}

void SpinBehaviorTester::activate()
{
  if (is_active_) {
    throw std::runtime_error("Trying to activate while already active");
    return;
  }
  if (!make_fake_costmap_) {
    while (!initial_pose_received_) {
      RCLCPP_WARN(node_->get_logger(), "Initial pose not received");
      sendInitialPose();
      std::this_thread::sleep_for(100ms);
      rclcpp::spin_some(node_);
    }
  } else {
    sendFakeOdom(0.0);
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

  RCLCPP_INFO(this->node_->get_logger(), "Spin action server is ready");
  is_active_ = true;
}

void SpinBehaviorTester::deactivate()
{
  if (!is_active_) {
    throw std::runtime_error("Trying to deactivate while already inactive");
  }
  is_active_ = false;
}

bool SpinBehaviorTester::defaultSpinBehaviorTest(
  const float target_yaw,
  const double tolerance)
{
  if (!is_active_) {
    RCLCPP_ERROR(node_->get_logger(), "Not activated");
    return false;
  }

  // Sleep to let behavior server be ready for serving in multiple runs
  std::this_thread::sleep_for(5s);

  if (make_fake_costmap_) {
    sendFakeOdom(0.0);
  }

  auto goal_msg = Spin::Goal();
  goal_msg.target_yaw = target_yaw;

  // Intialize fake costmap
  if (make_fake_costmap_) {
    sendFakeCostmap(target_yaw);
    sendFakeOdom(0.0);
  }

  geometry_msgs::msg::PoseStamped initial_pose;
  if (!nav2_util::getCurrentPose(initial_pose, *tf_buffer_, "odom")) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Found current robot pose");
  RCLCPP_INFO(
    node_->get_logger(),
    "Init Yaw is %lf",
    fabs(tf2::getYaw(initial_pose.pose.orientation)));
  RCLCPP_INFO(node_->get_logger(), "Before sending goal");

  // Intialize fake costmap
  if (make_fake_costmap_) {
    sendFakeCostmap(target_yaw);
    sendFakeOdom(0.0);
  }

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  auto goal_handle_future = client_ptr_->async_send_goal(goal_msg);

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<Spin>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_ptr_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  if (make_fake_costmap_) {  // if we are faking the costmap, we will fake success.
    sendFakeOdom(0.0);
    sendFakeCostmap(target_yaw);
    RCLCPP_INFO(node_->get_logger(), "target_yaw %lf", target_yaw);
    // Slowly increment command yaw by increment to simulate the robot slowly spinning into place
    float step_size = tolerance / 4.0;
    for (float command_yaw = 0.0;
      abs(command_yaw) < abs(target_yaw);
      command_yaw = command_yaw + step_size)
    {
      sendFakeOdom(command_yaw);
      sendFakeCostmap(target_yaw);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    sendFakeOdom(target_yaw);
    sendFakeCostmap(target_yaw);
    RCLCPP_INFO(node_->get_logger(), "After sending goal");
  }
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    return false;
  }

  rclcpp_action::ClientGoalHandle<Spin>::WrappedResult wrapped_result = result_future.get();

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

  double goal_yaw = angles::normalize_angle(
    tf2::getYaw(initial_pose.pose.orientation) + target_yaw);
  double dyaw = angles::shortest_angular_distance(
    goal_yaw, tf2::getYaw(current_pose.pose.orientation));

  if (fabs(dyaw) > tolerance) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Init Yaw is %lf (tolerance %lf)",
      fabs(tf2::getYaw(initial_pose.pose.orientation)), tolerance);
    RCLCPP_ERROR(
      node_->get_logger(),
      "Current Yaw is %lf (tolerance %lf)",
      fabs(tf2::getYaw(current_pose.pose.orientation)), tolerance);
    RCLCPP_ERROR(
      node_->get_logger(),
      "Angular distance from goal is %lf (tolerance %lf)",
      fabs(dyaw), tolerance);
    return false;
  }

  return true;
}

void SpinBehaviorTester::sendFakeCostmap(float angle)
{
  nav2_msgs::msg::Costmap fake_costmap;

  fake_costmap.header.frame_id = "odom";
  fake_costmap.header.stamp = stamp_;
  fake_costmap.metadata.layer = "master";
  fake_costmap.metadata.resolution = .1;
  fake_costmap.metadata.size_x = 100;
  fake_costmap.metadata.size_y = 100;
  fake_costmap.metadata.origin.position.x = 0;
  fake_costmap.metadata.origin.position.y = 0;
  fake_costmap.metadata.origin.orientation.w = 1.0;
  float costmap_val = 0;
  for (int ix = 0; ix < 100; ix++) {
    for (int iy = 0; iy < 100; iy++) {
      if (abs(angle) > M_PI_2f32) {
        // fake obstacles in the way so we get failure due to potential collision
        costmap_val = 100;
      }
      fake_costmap.data.push_back(costmap_val);
    }
  }
  fake_costmap_publisher_->publish(fake_costmap);
}

void SpinBehaviorTester::sendInitialPose()
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

  publisher_->publish(pose);
  RCLCPP_INFO(node_->get_logger(), "Sent initial pose");
}

void SpinBehaviorTester::sendFakeOdom(float angle)
{
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = stamp_;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transformStamped);

  geometry_msgs::msg::PolygonStamped footprint;
  footprint.header.frame_id = "odom";
  footprint.header.stamp = stamp_;
  footprint.polygon.points.resize(4);
  footprint.polygon.points[0].x = 0.22;
  footprint.polygon.points[0].y = 0.22;
  footprint.polygon.points[1].x = 0.22;
  footprint.polygon.points[1].y = -0.22;
  footprint.polygon.points[2].x = -0.22;
  footprint.polygon.points[2].y = -0.22;
  footprint.polygon.points[3].x = -0.22;
  footprint.polygon.points[3].y = 0.22;
  fake_footprint_publisher_->publish(footprint);
}
void SpinBehaviorTester::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
{
  initial_pose_received_ = true;
}

}  // namespace nav2_system_tests
