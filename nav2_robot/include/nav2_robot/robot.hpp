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

#ifndef NAV2_ROBOT__ROBOT_HPP_
#define NAV2_ROBOT__ROBOT_HPP_

#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace nav2_robot
{

class Robot : public nav2_util::LifecycleHelperInterface
{
public:
  explicit Robot(nav2_util::LifecycleNode::SharedPtr node);
  explicit Robot(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    bool auto_start = false);
  Robot() = delete;
  ~Robot();

  bool getGlobalLocalizerPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose);
  bool getCurrentPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & robot_pose);
  bool getOdometry(nav_msgs::msg::Odometry::SharedPtr & robot_odom);
  std::string getName();
  void sendVelocity(geometry_msgs::msg::Twist twist);

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

protected:
  // Interfaces used for logging and creating publishers and subscribers
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  // Publishers and subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // Subscription callbacks
  void onPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

  // The current pose as received from the Pose subscription
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;

  // The odometry as received from the Odometry subscription
  nav_msgs::msg::Odometry::SharedPtr current_odom_;

  // Whether the subscriptions have been received
  bool initial_pose_received_{false};
  bool initial_odom_received_{false};

  // Information about the robot is contained in the URDF file
  std::string urdf_file_;
  urdf::Model model_;

  // Auto-start feature for non-lifecycle nodes
  bool auto_start_;
  void configure();
  void cleanup();
};

}  // namespace nav2_robot

#endif  // NAV2_ROBOT__ROBOT_HPP_
