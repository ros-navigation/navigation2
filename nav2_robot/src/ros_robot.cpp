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

#include <string>
#include <exception>
#include "urdf/model.h"
#include "nav2_robot/ros_robot.hpp"

namespace nav2_robot
{

RosRobot::RosRobot(rclcpp::Node * node)
: node_(node), initial_pose_received_(false), initial_odom_received_(false)
{
  // TODO(mhpanah): Topic names for pose and odom should should be configured with parameters
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", std::bind(&RosRobot::onPoseReceived, this, std::placeholders::_1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", std::bind(&RosRobot::onOdomReceived, this, std::placeholders::_1));

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmdVelocity", 1);
}

RosRobot::~RosRobot()
{
}

void
RosRobot::enterSafeState()
{
}

void
RosRobot::onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // TODO(mjeronimo): serialize access
  current_pose_ = msg;
  if (!initial_pose_received_) {
    initial_pose_received_ = true;
  }
}

void
RosRobot::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_ = msg;
  if (!initial_odom_received_) {
    initial_odom_received_ = true;
  }
}

bool
RosRobot::getCurrentPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
{
  if (!initial_pose_received_) {
    RCLCPP_WARN(node_->get_logger(), "Can't return current pose: Initial pose not yet received.");
    return false;
  } else {
    robot_pose = current_pose_;
  }
  return true;
}

bool
RosRobot::getCurrentVelocity(nav_msgs::msg::Odometry::SharedPtr & robot_velocity)
{
  if (!initial_odom_received_) {
    RCLCPP_WARN(node_->get_logger(), "Can't return current velocity: Initial odometry not yet"
      " received.");
    return false;
  } else {
    robot_velocity = current_velocity_;
  }
  return true;
}

// TODO(mhpanah): modify this method name and implementation to include robot types and Serial #(ID)
std::string
RosRobot::getRobotName()
{
  // Temporarily just returning a string until we enable parsing URDF file.
  return "turtlebot";
}

void
RosRobot::sendVelocity(geometry_msgs::msg::Twist twist)
{
  vel_pub_->publish(twist);
}

}  // namespace nav2_robot
