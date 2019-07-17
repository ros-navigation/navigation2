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

#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/qos.hpp"
#include "urdf/model.h"

namespace nav2_robot
{

Robot::Robot(nav2_util::LifecycleNode::SharedPtr node)
: Robot(node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface(),
    false)
{
}

Robot::Robot(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  bool auto_start)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  auto_start_(auto_start)
{
  RCLCPP_INFO(node_logging_->get_logger(), "Robot: Creating");

  if (auto_start_) {
    configure();
  }
}

Robot::~Robot()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Robot: Destroying");

  if (auto_start_) {
    cleanup();
  }
}

nav2_util::CallbackReturn
Robot::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_logging_->get_logger(), "Robot: Configuring");

  configure();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Robot::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_logging_->get_logger(), "Robot: Activating");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Robot::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_logging_->get_logger(), "Robot: Deactivating");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Robot::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_logging_->get_logger(), "Robot: Cleaning up");

  cleanup();

  return nav2_util::CallbackReturn::SUCCESS;
}

void
Robot::configure()
{
  // This class may be used from a module that comes up after AMCL has output
  // its initial pose, so that pose message uses durability TRANSIENT_LOCAL

  pose_sub_ = rclcpp::create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    node_topics_, "amcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&Robot::onPoseReceived, this, std::placeholders::_1));

  odom_sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
    node_topics_, "odom", rclcpp::SensorDataQoS(),
    std::bind(&Robot::onOdomReceived, this, std::placeholders::_1));

  vel_pub_ = rclcpp::create_publisher<geometry_msgs::msg::Twist>(
    node_topics_, "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)));
}

void
Robot::cleanup()
{
  pose_sub_.reset();
  odom_sub_.reset();
  vel_pub_.reset();
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
    RCLCPP_DEBUG(node_logging_->get_logger(),
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
    RCLCPP_DEBUG(node_logging_->get_logger(),
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
