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

#ifndef RECOVERIES__SPIN__SPIN_RECOVERY_TESTER_HPP_
#define RECOVERIES__SPIN__SPIN_RECOVERY_TESTER_HPP_

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "angles/angles.h"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_thread.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_system_tests
{

class SpinRecoveryTester
{
public:
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  SpinRecoveryTester();
  ~SpinRecoveryTester();

  // Runs a single test with given target yaw
  bool defaultSpinRecoveryTest(
    float target_yaw,
    double tolerance = 0.1);

  void activate();

  void deactivate();

  bool isActive() const
  {
    return is_active_;
  }

private:
  void sendInitialPose();

  void sendFakeCostmap(float angle);
  void sendFakeOdom(float angle);

  void amclPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

  bool is_active_;
  bool initial_pose_received_;
  bool make_fake_costmap_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Node::SharedPtr node_;

  // Publisher to publish initial pose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

  // Publisher to publish fake costmap raw
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr fake_costmap_publisher_;

  // Publisher to publish fake costmap footprint
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr fake_footprint_publisher_;

  // Subscriber for amcl pose
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;

  // Action client to call spin action
  rclcpp_action::Client<Spin>::SharedPtr client_ptr_;
};

}  // namespace nav2_system_tests

#endif  // RECOVERIES__SPIN__SPIN_RECOVERY_TESTER_HPP_
