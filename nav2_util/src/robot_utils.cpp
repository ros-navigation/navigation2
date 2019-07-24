// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
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

#include <nav2_util/robot_utils.hpp>

namespace nav2_util
{

VelocityPublisher::VelocityPublisher(
  rclcpp::Node::SharedPtr & node,
  const std::string topic)
{
  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(topic, 1);
}

void
VelocityPublisher::publishCommand(const geometry_msgs::msg::Twist & cmd_vel)
{
  vel_pub_->publish(cmd_vel);
}

RobotStateHelper::RobotStateHelper(
  rclcpp::Node::SharedPtr & node,
  const std::string odom_topic)
: node_(node)
{
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", std::bind(&RobotStateHelper::onPoseReceived, this, std::placeholders::_1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, std::bind(&RobotStateHelper::onOdomReceived, this, std::placeholders::_1));

  initial_odom_received_ = false;
  initial_pose_received_ = false;
}

bool
RobotStateHelper::getOdometry(nav_msgs::msg::Odometry::SharedPtr & robot_odom)
{
  std::shared_lock<std::shared_timed_mutex> lock(state_mutex_);
  if (!initial_odom_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
      "getOdometry: Can't return current velocity: Initial odometry not yet received.");
    return false;
  }

  robot_odom = current_odom_;
  return true;
}

bool
RobotStateHelper::getCurrentPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose,
  const bool use_topic)
{
  if (use_topic) {
    return getGlobalLocalizerPose(robot_pose);
  }

  return getTfPose(robot_pose);
}

void
RobotStateHelper::onPoseReceived(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr robot_pose)
{
  std::unique_lock<std::shared_timed_mutex> lock(state_mutex_);
  current_pose_ = robot_pose;
  if (!initial_pose_received_) {
    initial_pose_received_ = true;
  }
}

void
RobotStateHelper::onOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::unique_lock<std::shared_timed_mutex> lock(state_mutex_);
  current_odom_ = msg;
  if (!initial_odom_received_) {
    initial_odom_received_ = true;
  }
}

bool
RobotStateHelper::getGlobalLocalizerPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
{
  std::shared_lock<std::shared_timed_mutex> lock(state_mutex_);
  if (!initial_pose_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Robot: Can't return current pose: Initial pose not yet received.");
    return false;
  }

  robot_pose = current_pose_;
  return true;
}

bool
RobotStateHelper::getTfPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & /*robot_pose*/)
{
  std::shared_lock<std::shared_timed_mutex> lock(state_mutex_);
  RCLCPP_DEBUG(node_->get_logger(), "getTfPose is not yet implemented.");
  return false;
}

} // end namespace nav2_util
