// Copyright (c) 2020 Samsung Research
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

#ifndef BEHAVIORS__ASSISTED_TELEOP__ASSISTED_TELEOP_BEHAVIOR_TESTER_HPP_
#define BEHAVIORS__ASSISTED_TELEOP__ASSISTED_TELEOP_BEHAVIOR_TESTER_HPP_

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_system_tests
{

class AssistedTeleopBehaviorTester
{
public:
  using AssistedTeleop = nav2_msgs::action::AssistedTeleop;

  AssistedTeleopBehaviorTester();
  ~AssistedTeleopBehaviorTester();

  // Runs a single test with given target yaw
  bool defaultAssistedTeleopTest(
    const float lin_vel,
    const float ang_vel);

  void activate();

  void deactivate();

  bool isActive() const
  {
    return is_active_;
  }

private:
  void sendInitialPose();

  void amclPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

  void filteredVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);

  unsigned int counter_;
  bool is_active_;
  bool initial_pose_received_;
  rclcpp::Time stamp_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr preempt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr filtered_vel_sub_;

  // Action client to call AssistedTeleop action
  rclcpp_action::Client<AssistedTeleop>::SharedPtr client_ptr_;

  // collision checking
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace nav2_system_tests

#endif  // BEHAVIORS__ASSISTED_TELEOP__ASSISTED_TELEOP_BEHAVIOR_TESTER_HPP_
