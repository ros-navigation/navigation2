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

#ifndef BEHAVIORS__BACKUP__BACKUP_BEHAVIOR_TESTER_HPP_
#define BEHAVIORS__BACKUP__BACKUP_BEHAVIOR_TESTER_HPP_

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "angles/angles.h"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_thread.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_system_tests
{

class BackupBehaviorTester
{
public:
  using BackUp = nav2_msgs::action::BackUp;
  using GoalHandleBackup = rclcpp_action::ClientGoalHandle<BackUp>;

  BackupBehaviorTester();
  ~BackupBehaviorTester();

  // Runs a single test with given target yaw
  bool defaultBackupBehaviorTest(
    const BackUp::Goal goal_msg,
    const double tolerance);

  void activate();

  void deactivate();

  bool isActive() const
  {
    return is_active_;
  }

private:
  void sendInitialPose();

  void amclPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

  bool is_active_;
  bool initial_pose_received_;
  rclcpp::Time stamp_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Node::SharedPtr node_;

  // Publisher to publish initial pose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

  // Subscriber for amcl pose
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;

  // Action client to call Backup action
  rclcpp_action::Client<BackUp>::SharedPtr client_ptr_;
};

}  // namespace nav2_system_tests

#endif  // BEHAVIORS__BACKUP__BACKUP_BEHAVIOR_TESTER_HPP_
