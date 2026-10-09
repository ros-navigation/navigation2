// Copyright (c) 2026 Open Navigation LLC
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

#include "gtest/gtest.h"
#include "nav2_ros_common/tf_utils.hpp"

class TfUtilsTest : public ::testing::Test
{
protected:
  void addTransform(int seconds, double x, bool is_static = false)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.header.stamp = rclcpp::Time(seconds, 0, RCL_ROS_TIME);
    transform.transform.translation.x = x;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 3.0;
    transform.transform.rotation.z = 0.6;
    transform.transform.rotation.w = 0.8;
    ASSERT_TRUE(buffer_.setTransform(transform, "test", is_static));
  }

  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  nav2::TransformBuffer buffer_{clock_};
  const rclcpp::Time now_{10, 0, RCL_ROS_TIME};
};

TEST_F(TfUtilsTest, LatestStalenessAndDisabledCheck)
{
  addTransform(8, 1.0);
  geometry_msgs::msg::TransformStamped result;
  result.header.frame_id = "unchanged";
  const auto original = result;
  EXPECT_FALSE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result, 1.0));
  EXPECT_EQ(result, original);
  ASSERT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result, 2.0));
  EXPECT_EQ(result.header.stamp.sec, 8);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 1.0);
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result));
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result, 0.0));
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result, -1.0));
  const auto previous = result;
  EXPECT_FALSE(nav2::getLatestTransform(buffer_, "odom", "missing", now_, result));
  EXPECT_EQ(result, previous);
}

TEST_F(TfUtilsTest, PoseTransformConversionsPreserveHeaderAndGeometry)
{
  addTransform(9, 1.0);
  geometry_msgs::msg::TransformStamped transform;
  ASSERT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, transform));

  const auto pose = nav2::transformToPoseStamped(transform);
  EXPECT_EQ(pose.header, transform.header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 3.0);
  EXPECT_EQ(pose.pose.orientation, transform.transform.rotation);
  EXPECT_EQ(nav2::poseToTransformStamped(pose, transform.child_frame_id), transform);
  EXPECT_EQ(nav2::poseToTransformStamped(pose, "other_child").child_frame_id, "other_child");
}

TEST_F(TfUtilsTest, LatestStaticAndFutureTransforms)
{
  addTransform(1, 1.0, true);
  geometry_msgs::msg::TransformStamped result;
  ASSERT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result, 0.1));
  EXPECT_EQ(result.header.stamp.sec, 0);
  EXPECT_EQ(result.header.stamp.nanosec, 0u);

  buffer_.clear();
  addTransform(11, 2.0);
  ASSERT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, result, 0.1));
  EXPECT_EQ(result.header.stamp.sec, 11);
}

TEST_F(TfUtilsTest, SameFrameIdentity)
{
  geometry_msgs::msg::TransformStamped result;
  result.transform.translation.x = 100.0;
  ASSERT_TRUE(nav2::getLatestTransform(buffer_, "base_link", "base_link", now_, result, 0.1));
  EXPECT_EQ(result.header.frame_id, "base_link");
  EXPECT_EQ(result.child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(result.header.stamp), now_);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result.transform.rotation.w, 1.0);

  const rclcpp::Time stamp(5, 123, RCL_ROS_TIME);
  ASSERT_TRUE(nav2::getStampedTransform(buffer_, "base_link", "base_link", stamp, result));
  EXPECT_EQ(rclcpp::Time(result.header.stamp), stamp);
}

TEST_F(TfUtilsTest, LatestPosePreservesHeaderAndGeometry)
{
  addTransform(9, 1.0);
  geometry_msgs::msg::PoseStamped pose;
  ASSERT_TRUE(nav2::getLatestPose(buffer_, "odom", "base_link", now_, pose, 1.0));
  EXPECT_EQ(pose.header.frame_id, "odom");
  EXPECT_EQ(pose.header.stamp.sec, 9);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.z, 0.6);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 0.8);
  const auto previous = pose;
  EXPECT_FALSE(nav2::getLatestPose(buffer_, "odom", "base_link", now_, pose, 0.5));
  EXPECT_EQ(pose, previous);
  EXPECT_FALSE(nav2::getLatestPose(buffer_, "odom", "missing", now_, pose));
  EXPECT_EQ(pose, previous);
  EXPECT_TRUE(nav2::getLatestPose(buffer_, "odom", "base_link", now_, pose, -1.0));
}

TEST_F(TfUtilsTest, StampedLookupInterpolatesHistoricalTransform)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  geometry_msgs::msg::TransformStamped result;
  const rclcpp::Time stamp(3, 0, RCL_ROS_TIME);
  ASSERT_TRUE(nav2::getStampedTransform(buffer_, "odom", "base_link", stamp, result));
  EXPECT_EQ(rclcpp::Time(result.header.stamp), stamp);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 3.0);
  const auto previous = result;
  EXPECT_FALSE(nav2::getStampedTransform(buffer_, "odom", "base_link", now_, result));
  EXPECT_EQ(result, previous);
  EXPECT_FALSE(nav2::getStampedTransform(buffer_, "odom", "missing", stamp, result));
  EXPECT_EQ(result, previous);
}

TEST_F(TfUtilsTest, ZeroStampLooksUpLatestTransform)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  geometry_msgs::msg::TransformStamped result;
  const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  ASSERT_TRUE(nav2::getStampedTransform(buffer_, "odom", "base_link", zero, result));
  EXPECT_EQ(result.header.frame_id, "odom");
  EXPECT_EQ(result.child_frame_id, "base_link");
  EXPECT_EQ(result.header.stamp.sec, 4);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 4.0);
  const auto previous = result;
  EXPECT_FALSE(nav2::getStampedTransform(buffer_, "odom", "missing", zero, result));
  EXPECT_EQ(result, previous);

  ASSERT_TRUE(nav2::getStampedTransform(buffer_, "base_link", "base_link", zero, result));
  EXPECT_EQ(rclcpp::Time(result.header.stamp), zero);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result.transform.rotation.w, 1.0);
}

TEST_F(TfUtilsTest, DifferentTimesInSameFrameAreNotIdentity)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  geometry_msgs::msg::TransformStamped result;
  const rclcpp::Time source_time(2, 0, RCL_ROS_TIME);
  const rclcpp::Time target_time(4, 0, RCL_ROS_TIME);
  ASSERT_TRUE(nav2::getStampedTransform(
      buffer_, "base_link", target_time, "base_link", source_time, "odom", result));
  EXPECT_EQ(result.header.frame_id, "base_link");
  EXPECT_EQ(result.child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(result.header.stamp), target_time);
  // Translation (-2, 0, 0) rotated by the inverse of (z=0.6, w=0.8).
  EXPECT_NEAR(result.transform.translation.x, -0.56, 1e-12);
  EXPECT_NEAR(result.transform.translation.y, 1.92, 1e-12);
  EXPECT_NEAR(result.transform.rotation.w, 1.0, 1e-12);
  const auto previous = result;
  EXPECT_FALSE(nav2::getStampedTransform(
      buffer_, "base_link", now_, "base_link", source_time, "odom", result));
  EXPECT_EQ(result, previous);
  EXPECT_FALSE(nav2::getStampedTransform(
      buffer_, "base_link", target_time, "base_link", source_time, "missing", result));
  EXPECT_EQ(result, previous);
}

TEST_F(TfUtilsTest, ZeroSourceAndTargetTimesLookUpLatestTransforms)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  geometry_msgs::msg::TransformStamped result;
  const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  const rclcpp::Time source_time(2, 0, RCL_ROS_TIME);
  const rclcpp::Time target_time(4, 0, RCL_ROS_TIME);

  ASSERT_TRUE(nav2::getStampedTransform(
      buffer_, "base_link", zero, "base_link", source_time, "odom", result));
  EXPECT_EQ(rclcpp::Time(result.header.stamp), target_time);
  EXPECT_NEAR(result.transform.translation.x, -0.56, 1e-12);
  EXPECT_NEAR(result.transform.translation.y, 1.92, 1e-12);

  ASSERT_TRUE(nav2::getStampedTransform(
      buffer_, "base_link", target_time, "base_link", zero, "odom", result));
  EXPECT_EQ(rclcpp::Time(result.header.stamp), target_time);
  EXPECT_NEAR(result.transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-12);
  EXPECT_NEAR(result.transform.rotation.w, 1.0, 1e-12);

  ASSERT_TRUE(nav2::getStampedTransform(
      buffer_, "base_link", zero, "base_link", zero, "odom", result));
  EXPECT_EQ(rclcpp::Time(result.header.stamp), target_time);
  EXPECT_NEAR(result.transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(result.transform.translation.y, 0.0, 1e-12);
  EXPECT_NEAR(result.transform.rotation.w, 1.0, 1e-12);
  const auto previous = result;
  EXPECT_FALSE(nav2::getStampedTransform(
      buffer_, "base_link", zero, "missing", zero, "odom", result));
  EXPECT_EQ(result, previous);
}
