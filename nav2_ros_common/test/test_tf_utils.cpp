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

  geometry_msgs::msg::PoseStamped makePose(int seconds) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = rclcpp::Time(seconds, 0, RCL_ROS_TIME);
    pose.pose.position.x = 1.0;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  nav2::TransformBuffer buffer_{clock_};
  const rclcpp::Time now_{10, 0, RCL_ROS_TIME};
};

TEST_F(TfUtilsTest, LatestStalenessAndDisabledCheck)
{
  addTransform(8, 1.0);
  EXPECT_EQ(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, 1.0), std::nullopt);
  auto result = nav2::getLatestTransform(buffer_, "odom", "base_link", now_, 2.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 8);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 1.0);
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link"));
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, 0.0));
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, -1.0));
  EXPECT_EQ(nav2::getLatestTransform(buffer_, "odom", "missing"), std::nullopt);
}

TEST_F(TfUtilsTest, PoseTransformConversionsPreserveHeaderAndGeometry)
{
  addTransform(9, 1.0);
  auto transform = nav2::getLatestTransform(buffer_, "odom", "base_link");
  ASSERT_TRUE(transform.has_value());

  const auto pose = nav2::transformToPoseStamped(*transform);
  EXPECT_EQ(pose.header, transform->header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 3.0);
  EXPECT_EQ(pose.pose.orientation, transform->transform.rotation);
  EXPECT_EQ(nav2::poseToTransformStamped(pose, transform->child_frame_id), *transform);
  EXPECT_EQ(nav2::poseToTransformStamped(pose, "other_child").child_frame_id, "other_child");
}

TEST_F(TfUtilsTest, LatestStaticAndFutureTransforms)
{
  addTransform(1, 1.0, true);
  auto result = nav2::getLatestTransform(buffer_, "odom", "base_link", now_, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 0);
  EXPECT_EQ(result->header.stamp.nanosec, 0u);

  buffer_.clear();
  addTransform(11, 2.0);
  result = nav2::getLatestTransform(buffer_, "odom", "base_link", now_, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 11);
}

TEST_F(TfUtilsTest, SameFrameIdentity)
{
  auto result = nav2::getLatestTransform(buffer_, "base_link", "base_link", now_, 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "base_link");
  EXPECT_EQ(result->child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(result->header.stamp), now_);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result->transform.rotation.w, 1.0);

  const rclcpp::Time stamp(5, 123, RCL_ROS_TIME);
  result = nav2::getStampedTransform(buffer_, "base_link", "base_link", stamp,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), stamp);
}

TEST_F(TfUtilsTest, LatestPosePreservesHeaderAndGeometry)
{
  addTransform(9, 1.0);
  auto pose = nav2::getLatestPose(buffer_, "odom", "base_link", now_, 1.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_EQ(pose->header.frame_id, "odom");
  EXPECT_EQ(pose->header.stamp.sec, 9);
  EXPECT_DOUBLE_EQ(pose->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose->pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(pose->pose.orientation.z, 0.6);
  EXPECT_DOUBLE_EQ(pose->pose.orientation.w, 0.8);
  EXPECT_EQ(nav2::getLatestPose(buffer_, "odom", "base_link", now_, 0.5), std::nullopt);
  EXPECT_EQ(nav2::getLatestPose(buffer_, "odom", "missing"), std::nullopt);
  EXPECT_TRUE(nav2::getLatestPose(buffer_, "odom", "base_link", now_, -1.0));
}

TEST_F(TfUtilsTest, StampedLookupInterpolatesHistoricalTransform)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time stamp(3, 0, RCL_ROS_TIME);
  auto result = nav2::getStampedTransform(buffer_, "odom", "base_link", stamp,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), stamp);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 3.0);
  EXPECT_EQ(
    nav2::getStampedTransform(buffer_, "odom", "base_link", now_, tf2::Duration::zero()),
    std::nullopt);
  EXPECT_EQ(
    nav2::getStampedTransform(buffer_, "odom", "missing", stamp, tf2::Duration::zero()),
    std::nullopt);
}

TEST_F(TfUtilsTest, ZeroStampLooksUpLatestTransform)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  auto result = nav2::getStampedTransform(buffer_, "odom", "base_link", zero,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "odom");
  EXPECT_EQ(result->child_frame_id, "base_link");
  EXPECT_EQ(result->header.stamp.sec, 4);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 4.0);
  EXPECT_EQ(
    nav2::getStampedTransform(buffer_, "odom", "missing", zero, tf2::Duration::zero()),
    std::nullopt);

  result = nav2::getStampedTransform(buffer_, "base_link", "base_link", zero,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), zero);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result->transform.rotation.w, 1.0);
}

TEST_F(TfUtilsTest, DifferentTimesInSameFrameAreNotIdentity)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time source_time(2, 0, RCL_ROS_TIME);
  const rclcpp::Time target_time(4, 0, RCL_ROS_TIME);
  auto result = nav2::getStampedTransform(
    buffer_, "base_link", target_time, "base_link", source_time, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "base_link");
  EXPECT_EQ(result->child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  // Translation (-2, 0, 0) rotated by the inverse of (z=0.6, w=0.8).
  EXPECT_NEAR(result->transform.translation.x, -0.56, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 1.92, 1e-12);
  EXPECT_NEAR(result->transform.rotation.w, 1.0, 1e-12);
  EXPECT_EQ(nav2::getStampedTransform(
    buffer_, "base_link", now_, "base_link", source_time, "odom", tf2::Duration::zero()),
    std::nullopt);
  EXPECT_EQ(nav2::getStampedTransform(
    buffer_, "base_link", target_time, "base_link", source_time, "missing", tf2::Duration::zero()),
    std::nullopt);
}

TEST_F(TfUtilsTest, ZeroSourceAndTargetTimesLookUpLatestTransforms)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  const rclcpp::Time source_time(2, 0, RCL_ROS_TIME);
  const rclcpp::Time target_time(4, 0, RCL_ROS_TIME);

  auto result = nav2::getStampedTransform(
    buffer_, "base_link", zero, "base_link", source_time, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  EXPECT_NEAR(result->transform.translation.x, -0.56, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 1.92, 1e-12);

  result = nav2::getStampedTransform(
    buffer_, "base_link", target_time, "base_link", zero, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  EXPECT_NEAR(result->transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.rotation.w, 1.0, 1e-12);

  result = nav2::getStampedTransform(
    buffer_, "base_link", zero, "base_link", zero, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  EXPECT_NEAR(result->transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.rotation.w, 1.0, 1e-12);
  EXPECT_EQ(nav2::getStampedTransform(
    buffer_, "base_link", zero, "missing", zero, "odom", tf2::Duration::zero()), std::nullopt);
}

TEST_F(TfUtilsTest, TransformStampedPoseInterpolatesAndPreservesInput)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const auto input = makePose(3);
  const auto original = input;
  const auto result = nav2::transformStampedPose(
    buffer_, "odom", input, tf2::Duration::zero());
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.frame_id, "odom");
  EXPECT_EQ(result->header.stamp, input.header.stamp);
  EXPECT_NEAR(result->pose.position.x, 3.28, 1e-12);
  EXPECT_NEAR(result->pose.position.y, 2.96, 1e-12);
  EXPECT_DOUBLE_EQ(result->pose.position.z, 3.0);
  EXPECT_NEAR(result->pose.orientation.z, 0.6, 1e-12);
  EXPECT_NEAR(result->pose.orientation.w, 0.8, 1e-12);
  EXPECT_EQ(input, original);
}

TEST_F(TfUtilsTest, TransformStampedPoseZeroStampSelectsLatestWithoutAgeCheck)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const auto result = nav2::transformStampedPose(
    buffer_, "odom", makePose(0), tf2::Duration::zero());
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 4);
  EXPECT_NEAR(result->pose.position.x, 4.28, 1e-12);
}

TEST_F(TfUtilsTest, TransformLatestPoseIgnoresInputStampAndChecksConfiguredAge)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const auto input = makePose(2);
  const auto original = input;
  auto result = nav2::transformLatestPose(buffer_, "odom", input, now_, 6.0);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 4);
  EXPECT_NEAR(result->pose.position.x, 4.28, 1e-12);
  EXPECT_EQ(nav2::transformLatestPose(buffer_, "odom", input, now_, 5.0), std::nullopt);
  EXPECT_TRUE(nav2::transformLatestPose(buffer_, "odom", input));
  EXPECT_TRUE(nav2::transformLatestPose(buffer_, "odom", input, now_, -1.0));
  EXPECT_EQ(input, original);
}

TEST_F(TfUtilsTest, TransformPoseSameFramePreservesEntireInputWithoutTF)
{
  const auto input = makePose(2);
  const auto stamped = nav2::transformStampedPose(
    buffer_, "base_link", input, tf2::Duration::zero());
  ASSERT_TRUE(stamped);
  EXPECT_EQ(*stamped, input);
  const auto latest = nav2::transformLatestPose(buffer_, "base_link", input, now_, 0.1);
  ASSERT_TRUE(latest);
  EXPECT_EQ(*latest, input);
}

TEST_F(TfUtilsTest, TransformPoseFailuresLeaveInputUnchanged)
{
  addTransform(2, 2.0);
  const auto input = makePose(3);
  const auto original = input;
  EXPECT_EQ(nav2::transformStampedPose(
      buffer_, "odom", input, tf2::Duration::zero()), std::nullopt);
  EXPECT_EQ(nav2::transformStampedPose(
      buffer_, "missing", input, tf2::Duration::zero()), std::nullopt);
  EXPECT_EQ(nav2::transformLatestPose(buffer_, "missing", input), std::nullopt);
  EXPECT_EQ(input, original);
}

TEST_F(TfUtilsTest, TransformLatestPoseAcceptsStaticAndFutureTransforms)
{
  addTransform(1, 1.0, true);
  auto result = nav2::transformLatestPose(buffer_, "odom", makePose(2), now_, 0.1);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 0);
  EXPECT_NEAR(result->pose.position.x, 1.28, 1e-12);
  buffer_.clear();
  addTransform(11, 2.0);
  result = nav2::transformLatestPose(buffer_, "odom", makePose(2), now_, 0.1);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 11);
}

TEST_F(TfUtilsTest, LatestLookupsWithoutCurrentTime)
{
  addTransform(1, 1.0);
  const auto transform = nav2::getLatestTransform(buffer_, "odom", "base_link");
  ASSERT_TRUE(transform);
  EXPECT_EQ(transform->header.stamp.sec, 1);
  const auto pose = nav2::getLatestPose(buffer_, "odom", "base_link");
  ASSERT_TRUE(pose);
  EXPECT_EQ(pose->header, transform->header);

  const auto identity = nav2::getLatestTransform(buffer_, "base_link", "base_link");
  ASSERT_TRUE(identity);
  EXPECT_EQ(identity->header.stamp.sec, 0);
  EXPECT_EQ(identity->header.stamp.nanosec, 0u);
  EXPECT_DOUBLE_EQ(identity->transform.rotation.w, 1.0);
  const auto identity_pose = nav2::getLatestPose(buffer_, "base_link", "base_link");
  ASSERT_TRUE(identity_pose);
  EXPECT_EQ(identity_pose->header, identity->header);
}

TEST_F(TfUtilsTest, TransformLatestPoseWithoutCurrentTime)
{
  addTransform(1, 1.0);
  const auto input = makePose(10);
  const auto result = nav2::transformLatestPose(buffer_, "odom", input);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 1);
  EXPECT_NEAR(result->pose.position.x, 1.28, 1e-12);

  const auto unchanged = nav2::transformLatestPose(buffer_, "base_link", input);
  ASSERT_TRUE(unchanged);
  EXPECT_EQ(*unchanged, input);
}

TEST_F(TfUtilsTest, PositiveStalenessThresholdRequiresCurrentTime)
{
  addTransform(9, 1.0);
  const auto input = makePose(9);
  EXPECT_EQ(nav2::getLatestTransform(
      buffer_, "odom", "base_link", std::nullopt, 1.0), std::nullopt);
  EXPECT_EQ(nav2::getLatestPose(
      buffer_, "odom", "base_link", std::nullopt, 1.0), std::nullopt);
  EXPECT_EQ(nav2::transformLatestPose(
      buffer_, "odom", input, std::nullopt, 1.0), std::nullopt);
  EXPECT_EQ(nav2::getLatestTransform(
      buffer_, "base_link", "base_link", std::nullopt, 1.0), std::nullopt);
  EXPECT_EQ(nav2::transformLatestPose(
      buffer_, "base_link", input, std::nullopt, 1.0), std::nullopt);
  EXPECT_TRUE(nav2::getLatestTransform(buffer_, "odom", "base_link", now_, 1.0));
}
