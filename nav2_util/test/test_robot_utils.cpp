// Copyright (c) 2020 Samsung Research
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

#include <memory>
#include <cmath>
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"

TEST(RobotUtils, LookupExceptionError)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2::LifecycleNode>("name");
  geometry_msgs::msg::PoseStamped global_pose;
  nav2::TransformBuffer tf(node->get_clock());
  ASSERT_FALSE(nav2_util::getCurrentPose(global_pose, tf, "map", "base_link", 0.1));
  global_pose.header.frame_id = "base_link";
  ASSERT_FALSE(nav2_util::transformPoseInTargetFrame(global_pose, global_pose, tf, "map", 0.1));
  rclcpp::shutdown();
}

TEST(RobotUtils, validateTwist)
{
  geometry_msgs::msg::Twist msg;
  EXPECT_TRUE(nav2_util::validateTwist(msg));

  msg.linear.x = NAN;
  EXPECT_FALSE(nav2_util::validateTwist(msg));
  msg.linear.x = 1;
  msg.linear.y = NAN;
  EXPECT_FALSE(nav2_util::validateTwist(msg));
  msg.linear.y = 1;
  msg.linear.z = NAN;
  EXPECT_FALSE(nav2_util::validateTwist(msg));

  msg.linear.z = 1;
  msg.angular.x = NAN;
  EXPECT_FALSE(nav2_util::validateTwist(msg));
  msg.angular.x = 1;
  msg.angular.y = NAN;
  EXPECT_FALSE(nav2_util::validateTwist(msg));
  msg.angular.y = 1;
  msg.angular.z = NAN;
  EXPECT_FALSE(nav2_util::validateTwist(msg));
}

TEST(RobotUtils, lookupTransformWithStalenessCheck)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  nav2::TransformBuffer tf(clock);
  const rclcpp::Time current_time(10, 0, RCL_ROS_TIME);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = rclcpp::Time(8, 0, RCL_ROS_TIME);
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  tf.setTransform(transform, "test", false);

  EXPECT_THROW(
    nav2_util::lookupTransformWithStalenessCheck(
      tf, "map", "base_link", tf2::durationFromSec(0.0), current_time, 1.0),
    tf2::ExtrapolationException);
  EXPECT_NO_THROW(
    nav2_util::lookupTransformWithStalenessCheck(
      tf, "map", "base_link", tf2::durationFromSec(0.0), current_time, 2.0));
  EXPECT_NO_THROW(
    nav2_util::lookupTransformWithStalenessCheck(
      tf, "map", "base_link", tf2::durationFromSec(0.0), current_time, 0.0));

  transform.header.frame_id = "map";
  transform.child_frame_id = "static_frame";
  tf.setTransform(transform, "test", true);
  EXPECT_NO_THROW(
    nav2_util::lookupTransformWithStalenessCheck(
      tf, "map", "static_frame", tf2::durationFromSec(0.0), current_time, 1.0));
}

TEST(RobotUtils, transformToPoseStamped)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp.sec = 12;
  transform.header.stamp.nanosec = 34;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.x = 0.1;
  transform.transform.rotation.y = 0.2;
  transform.transform.rotation.z = 0.3;
  transform.transform.rotation.w = 0.4;

  const auto pose = nav2_util::transformToPoseStamped(transform);

  EXPECT_EQ(pose.header.frame_id, transform.header.frame_id);
  EXPECT_EQ(pose.header.stamp, transform.header.stamp);
  EXPECT_EQ(pose.pose.position.x, transform.transform.translation.x);
  EXPECT_EQ(pose.pose.position.y, transform.transform.translation.y);
  EXPECT_EQ(pose.pose.position.z, transform.transform.translation.z);
  EXPECT_EQ(pose.pose.orientation, transform.transform.rotation);
}
