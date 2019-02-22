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

#include "nav2_robot/robot.hpp"

#include <string>
#include <exception>
#include "urdf/model.h"

namespace nav2_robot
{

Robot::Robot(nav2_lifecycle::LifecycleNode::SharedPtr node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Robot: Creating");
}

Robot::~Robot()
{
  RCLCPP_INFO(node_->get_logger(), "Robot: Destroying");
}

nav2_lifecycle::CallbackReturn
Robot::onConfigure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Robot: onConfigure");

  // This class may be used from a module that comes up after AMCL has output
  // its initial pose, so that pose message uses durability TRANSIENT_LOCAL 
  rmw_qos_profile_t pose_qos_profile = rmw_qos_profile_default;
  pose_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", std::bind(&Robot::onPoseReceived, this, std::placeholders::_1), pose_qos_profile);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", std::bind(&Robot::onOdomReceived, this, std::placeholders::_1), rmw_qos_profile_sensor_data);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
Robot::onActivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Robot: onActivate");

  vel_pub_->on_activate();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
Robot::onDeactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Robot: onDeactivate");

  vel_pub_->on_deactivate();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
Robot::onCleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Robot: onCleanup");

  pose_sub_.reset();
  odom_sub_.reset();
  vel_pub_.reset();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

void
Robot::onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // TODO(mjeronimo): serialize access
  current_pose_ = msg;
  if (!initial_pose_received_) {
    initial_pose_received_ = true;
  }
}

void
Robot::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
  if (!initial_odom_received_) {
    initial_odom_received_ = true;
  }
}

bool
Robot::getGlobalLocalizerPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
{
  if (!initial_pose_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Robot: Can't return current pose: Initial pose not yet received.");
    return false;
  }

  robot_pose = current_pose_;
  return true;
}

// TODO(mhpanah): We should get current pose from transforms.
bool
Robot::getCurrentPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose)
{
  return getGlobalLocalizerPose(robot_pose);
}

bool
Robot::getOdometry(nav_msgs::msg::Odometry::SharedPtr & robot_odom)
{
  if (!initial_odom_received_) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Robot: Can't return current velocity: Initial odometry not yet received.");
    return false;
  }

  robot_odom = current_odom_;
  return true;
}

// TODO(mhpanah): modify this method name and implementation to include robot types and Serial #(ID)
std::string
Robot::getName()
{
  // Temporarily just returning a string until we enable parsing URDF file.
  return "turtlebot";
}

void
Robot::sendVelocity(geometry_msgs::msg::Twist twist)
{
  vel_pub_->publish(twist);
}

}  // namespace nav2_robot
