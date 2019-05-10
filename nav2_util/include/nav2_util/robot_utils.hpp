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

#ifndef NAV2_UTIL__ROBOT_UTILS_HPP_
#define NAV2_UTIL__ROBOT_UTILS_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace nav2_util
{

class VelocityPublisher
{
public:
  VelocityPublisher(rclcpp::Node::SharedPtr & node,
    const std::string topic = std::string("/cmd_vel"));

  void publishCommand(const geometry_msgs::msg::Twist& cmd_vel);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

class RobotStateHelper
{
public:
  RobotStateHelper(rclcpp::Node::SharedPtr & node,
    const std::string odom_topic = std::string("/odom"));

  bool getOdometry();

  bool getCurrentPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose,
    const bool use_topic = false);

private:
  void onPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose);
  void onOdomReceived();

  bool getGlobalLocalizerPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose);
  bool getTfPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_mutex state_mutex_;
  bool initial_pose_received_, initial_odom_received_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
}

} // end namespace nav2_util
