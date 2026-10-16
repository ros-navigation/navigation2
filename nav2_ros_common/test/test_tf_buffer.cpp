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

#include <limits>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rcl/time.h"
#include "nav2_ros_common/tf_buffer.hpp"

class TransformBufferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    setTime(now_);
  }

  void setTime(const rclcpp::Time & time)
  {
    ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), time.nanoseconds()),
      RCL_RET_OK);
  }

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
    return pose;
  }

  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer::SharedPtr tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  nav2::TransformBuffer buffer_{tf_buffer_, clock_};
  const rclcpp::Time now_{10, 0, RCL_ROS_TIME};
};

TEST_F(TransformBufferTest, LatestStalenessAndDisabledCheck)
{
  addTransform(8, 1.0);
  EXPECT_EQ(buffer_.getLatestTransform("odom", "base_link", 1.0), std::nullopt);
  auto result = buffer_.getLatestTransform("odom", "base_link", 2.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 8);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 1.0);
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link"));
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link", 0.0));
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link", -1.0));
  EXPECT_EQ(buffer_.getLatestTransform("odom", "missing"), std::nullopt);
}

TEST_F(TransformBufferTest, QuietLookupsRespectTimestamp)
{
  addTransform(8, 1.0);
  addTransform(10, 3.0);
  testing::internal::CaptureStderr();
  EXPECT_FALSE(buffer_.getLatestTransform("odom", "missing", 0.0,
    nav2::Log::Disabled).has_value());
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link", 0.0,
    nav2::Log::Disabled).has_value());
  EXPECT_TRUE(buffer_.getStampedTransform("odom", "base_link", rclcpp::Time(9, 0),
    tf2::Duration::zero(), nav2::Log::Disabled).has_value());
  EXPECT_FALSE(buffer_.getStampedTransform("odom", "base_link", rclcpp::Time(7, 0),
    tf2::Duration::zero(), nav2::Log::Disabled).has_value());
  EXPECT_FALSE(buffer_.getStampedTransform(
      "odom", "base_link", now_, tf2::durationFromSec(-0.1), nav2::Log::Disabled).has_value());
  EXPECT_EQ(testing::internal::GetCapturedStderr(), "");
}

TEST_F(TransformBufferTest, PoseTransformConversionsPreserveHeaderAndGeometry)
{
  addTransform(9, 1.0);
  auto transform = buffer_.getLatestTransform("odom", "base_link");
  ASSERT_TRUE(transform.has_value());

  const auto pose = nav2::TransformBuffer::transformToPoseStamped(*transform);
  EXPECT_EQ(pose.header, transform->header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 3.0);
  EXPECT_EQ(pose.pose.orientation, transform->transform.rotation);
  EXPECT_EQ(
    nav2::TransformBuffer::poseToTransformStamped(pose, transform->child_frame_id), *transform);
  EXPECT_EQ(
    nav2::TransformBuffer::poseToTransformStamped(pose, "other_child").child_frame_id,
    "other_child");
}

TEST_F(TransformBufferTest, LatestStaticAndFutureTransforms)
{
  addTransform(1, 1.0, true);
  auto result = buffer_.getLatestTransform("odom", "base_link", 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 0);
  EXPECT_EQ(result->header.stamp.nanosec, 0u);

  buffer_.clear();
  addTransform(11, 2.0);
  result = buffer_.getLatestTransform("odom", "base_link", 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 11);
}

TEST_F(TransformBufferTest, SameFrameIdentity)
{
  auto result = buffer_.getLatestTransform("base_link", "base_link", 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "base_link");
  EXPECT_EQ(result->child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(result->header.stamp), now_);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result->transform.rotation.w, 1.0);

  const rclcpp::Time stamp(5, 123, RCL_ROS_TIME);
  result = buffer_.getStampedTransform("base_link", "base_link", stamp,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), stamp);
}

TEST_F(TransformBufferTest, LatestPosePreservesHeaderAndGeometry)
{
  addTransform(9, 1.0);
  auto pose = buffer_.getLatestPose("odom", "base_link", 1.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_EQ(pose->header.frame_id, "odom");
  EXPECT_EQ(pose->header.stamp.sec, 9);
  EXPECT_DOUBLE_EQ(pose->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose->pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(pose->pose.orientation.z, 0.6);
  EXPECT_DOUBLE_EQ(pose->pose.orientation.w, 0.8);
  EXPECT_EQ(buffer_.getLatestPose("odom", "base_link", 0.5), std::nullopt);
  EXPECT_EQ(buffer_.getLatestPose("odom", "missing"), std::nullopt);
  EXPECT_TRUE(buffer_.getLatestPose("odom", "base_link", -1.0));
}

TEST_F(TransformBufferTest, StampedLookupInterpolatesHistoricalTransform)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time stamp(3, 0, RCL_ROS_TIME);
  auto result = buffer_.getStampedTransform("odom", "base_link", stamp,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), stamp);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 3.0);
  EXPECT_EQ(
    buffer_.getStampedTransform("odom", "base_link", now_, tf2::Duration::zero()),
    std::nullopt);
  EXPECT_EQ(
    buffer_.getStampedTransform("odom", "missing", stamp, tf2::Duration::zero()),
    std::nullopt);
}

TEST_F(TransformBufferTest, ZeroStampLooksUpLatestTransform)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  auto result = buffer_.getStampedTransform("odom", "base_link", zero,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "odom");
  EXPECT_EQ(result->child_frame_id, "base_link");
  EXPECT_EQ(result->header.stamp.sec, 4);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 4.0);
  EXPECT_EQ(
    buffer_.getStampedTransform("odom", "missing", zero, tf2::Duration::zero()),
    std::nullopt);

  result = buffer_.getStampedTransform("base_link", "base_link", zero,
    tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), zero);
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result->transform.rotation.w, 1.0);
}

TEST_F(TransformBufferTest, DifferentTimesInSameFrameAreNotIdentity)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time source_time(2, 0, RCL_ROS_TIME);
  const rclcpp::Time target_time(4, 0, RCL_ROS_TIME);
  auto result = buffer_.getStampedTransform(
    "base_link", target_time, "base_link", source_time, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "base_link");
  EXPECT_EQ(result->child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  // Translation (-2, 0, 0) rotated by the inverse of (z=0.6, w=0.8).
  EXPECT_NEAR(result->transform.translation.x, -0.56, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 1.92, 1e-12);
  EXPECT_NEAR(result->transform.rotation.w, 1.0, 1e-12);
  EXPECT_EQ(buffer_.getStampedTransform(
    "base_link", now_, "base_link", source_time, "odom", tf2::Duration::zero()),
    std::nullopt);
  EXPECT_EQ(buffer_.getStampedTransform(
    "base_link", target_time, "base_link", source_time, "missing", tf2::Duration::zero()),
    std::nullopt);
}

TEST_F(TransformBufferTest, ZeroSourceAndTargetTimesLookUpLatestTransforms)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  const rclcpp::Time source_time(2, 0, RCL_ROS_TIME);
  const rclcpp::Time target_time(4, 0, RCL_ROS_TIME);

  auto result = buffer_.getStampedTransform(
    "base_link", zero, "base_link", source_time, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  EXPECT_NEAR(result->transform.translation.x, -0.56, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 1.92, 1e-12);

  result = buffer_.getStampedTransform(
    "base_link", target_time, "base_link", zero, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  EXPECT_NEAR(result->transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.rotation.w, 1.0, 1e-12);

  result = buffer_.getStampedTransform(
    "base_link", zero, "base_link", zero, "odom", tf2::Duration::zero());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp), target_time);
  EXPECT_NEAR(result->transform.translation.x, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.translation.y, 0.0, 1e-12);
  EXPECT_NEAR(result->transform.rotation.w, 1.0, 1e-12);
  EXPECT_EQ(buffer_.getStampedTransform(
    "base_link", zero, "missing", zero, "odom", tf2::Duration::zero()), std::nullopt);
}

TEST_F(TransformBufferTest, GetStampedPoseInterpolatesAndPreservesInput)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const auto input = makePose(3);
  const auto original = input;
  const auto result = buffer_.transformStampedPose("odom", input, tf2::Duration::zero());
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

TEST_F(TransformBufferTest, GetStampedPoseZeroStampSelectsLatestWithoutAgeCheck)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const auto result = buffer_.transformStampedPose("odom", makePose(0), tf2::Duration::zero());
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 4);
  EXPECT_NEAR(result->pose.position.x, 4.28, 1e-12);
}

TEST_F(TransformBufferTest, TransformPoseSameFramePreservesEntireInputWithoutTF)
{
  const auto input = makePose(2);
  const auto stamped = buffer_.transformStampedPose("base_link", input, tf2::Duration::zero());
  ASSERT_TRUE(stamped);
  EXPECT_EQ(*stamped, input);
}

TEST_F(TransformBufferTest, TransformPoseFailuresLeaveInputUnchanged)
{
  addTransform(2, 2.0);
  const auto input = makePose(3);
  const auto original = input;
  EXPECT_EQ(buffer_.transformStampedPose("odom", input, tf2::Duration::zero()), std::nullopt);
  EXPECT_EQ(buffer_.transformStampedPose("missing", input, tf2::Duration::zero()), std::nullopt);
  EXPECT_EQ(input, original);
}

TEST_F(TransformBufferTest, LatestLookupsUseStoredClock)
{
  addTransform(1, 1.0);
  const auto transform = buffer_.getLatestTransform("odom", "base_link");
  ASSERT_TRUE(transform);
  EXPECT_EQ(transform->header.stamp.sec, 1);
  const auto pose = buffer_.getLatestPose("odom", "base_link");
  ASSERT_TRUE(pose);
  EXPECT_EQ(pose->header, transform->header);

  const auto identity = buffer_.getLatestTransform("base_link", "base_link");
  ASSERT_TRUE(identity);
  EXPECT_EQ(rclcpp::Time(identity->header.stamp), clock_->now());
  EXPECT_DOUBLE_EQ(identity->transform.rotation.w, 1.0);
  const auto identity_pose = buffer_.getLatestPose("base_link", "base_link");
  ASSERT_TRUE(identity_pose);
  EXPECT_EQ(identity_pose->header, identity->header);
}

TEST_F(TransformBufferTest, StalenessTracksClockAdvancement)
{
  addTransform(9, 1.0);
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link", 1.0));
  EXPECT_TRUE(buffer_.getLatestPose("odom", "base_link", 1.0));
  setTime(rclcpp::Time(11, 0, RCL_ROS_TIME));
  EXPECT_EQ(buffer_.getLatestTransform("odom", "base_link", 1.0), std::nullopt);
  EXPECT_EQ(buffer_.getLatestPose("odom", "base_link", 1.0), std::nullopt);
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link", 2.0));
  const auto identity = buffer_.getLatestTransform("base_link", "base_link", 1.0);
  ASSERT_TRUE(identity);
  EXPECT_EQ(rclcpp::Time(identity->header.stamp), clock_->now());
}

TEST_F(TransformBufferTest, RejectsNullBuffer)
{
  EXPECT_THROW(nav2::TransformBuffer(tf2_ros::Buffer::SharedPtr{}, clock_), std::invalid_argument);
}

TEST_F(TransformBufferTest, RejectsNullClock)
{
  EXPECT_THROW(nav2::TransformBuffer(rclcpp::Clock::SharedPtr{}), std::invalid_argument);
  EXPECT_THROW(nav2::TransformBuffer(tf_buffer_, rclcpp::Clock::SharedPtr{}),
    std::invalid_argument);
}

TEST_F(TransformBufferTest, OwnedBufferUsesProvidedClockForStaleness)
{
  nav2::TransformBuffer owned(clock_);
  ASSERT_TRUE(owned.setTransform(
      nav2::TransformBuffer::poseToTransformStamped(makePose(9), "sensor"), "test"));
  EXPECT_TRUE(owned.getLatestTransform("base_link", "sensor", 1.0));
  setTime(rclcpp::Time(11, 0, RCL_ROS_TIME));
  EXPECT_EQ(owned.getLatestTransform("base_link", "sensor", 1.0), std::nullopt);
}

TEST_F(TransformBufferTest, RetainsBufferOwnership)
{
  addTransform(9, 1.0);
  tf_buffer_.reset();
  EXPECT_TRUE(buffer_.getLatestTransform("odom", "base_link"));
}

TEST_F(TransformBufferTest, InversePreservesTimestampAndExchangesFrames)
{
  addTransform(9, 1.0);
  const auto transform = buffer_.getLatestTransform("odom", "base_link");
  ASSERT_TRUE(transform);
  const auto inverse = nav2::TransformBuffer::invertTransform(*transform);
  EXPECT_EQ(inverse.header.frame_id, transform->child_frame_id);
  EXPECT_EQ(inverse.child_frame_id, transform->header.frame_id);
  EXPECT_EQ(inverse.header.stamp, transform->header.stamp);
  const auto restored = nav2::TransformBuffer::invertTransform(inverse);
  EXPECT_NEAR(restored.transform.translation.x, transform->transform.translation.x, 1e-12);
  EXPECT_NEAR(restored.transform.translation.y, transform->transform.translation.y, 1e-12);
  EXPECT_NEAR(restored.transform.translation.z, transform->transform.translation.z, 1e-12);
  EXPECT_NEAR(restored.transform.rotation.z, transform->transform.rotation.z, 1e-12);
  EXPECT_NEAR(restored.transform.rotation.w, transform->transform.rotation.w, 1e-12);
}

TEST_F(TransformBufferTest, RejectsNegativeToleranceIncludingIdentity)
{
  addTransform(9, 1.0);
  const auto negative_tolerance = tf2::durationFromSec(-0.1);
  EXPECT_EQ(buffer_.getStampedTransform("odom", "base_link", now_, negative_tolerance),
    std::nullopt);
  EXPECT_EQ(buffer_.getStampedTransform("base_link", "base_link", now_, negative_tolerance),
    std::nullopt);
  EXPECT_EQ(buffer_.getStampedTransform(
      "base_link", now_, "base_link", now_, "odom", negative_tolerance), std::nullopt);
  EXPECT_EQ(buffer_.transformStampedPose("odom", makePose(9), negative_tolerance), std::nullopt);
  EXPECT_EQ(buffer_.transformStampedPose("base_link", makePose(9), negative_tolerance),
    std::nullopt);
}

TEST_F(TransformBufferTest, RejectsNonfiniteStalenessThreshold)
{
  addTransform(9, 1.0);
  EXPECT_EQ(buffer_.getLatestTransform(
      "odom", "base_link", std::numeric_limits<double>::quiet_NaN()), std::nullopt);
  EXPECT_EQ(buffer_.getLatestPose(
      "odom", "base_link", std::numeric_limits<double>::infinity()), std::nullopt);
}

TEST_F(TransformBufferTest, LogsFailuresWithConfiguredLoggerAndFrameContext)
{
  const nav2::TransformBuffer buffer(tf_buffer_, clock_, rclcpp::get_logger("tf_buffer_test"));
  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer.getLatestPose("odom", "missing"), std::nullopt);
  const auto lookup_log = testing::internal::GetCapturedStderr();
  EXPECT_NE(lookup_log.find("tf_buffer_test"), std::string::npos);
  EXPECT_NE(lookup_log.find("Failed to get transform from 'missing' to 'odom'"), std::string::npos);

  addTransform(8, 1.0);
  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer.getLatestTransform("odom", "base_link", 1.0), std::nullopt);
  const auto stale_log = testing::internal::GetCapturedStderr();
  EXPECT_NE(stale_log.find("Stale transform from 'base_link' to 'odom'"), std::string::npos);
  EXPECT_NE(stale_log.find("2.000000 s"), std::string::npos);

  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer.getLatestTransform(
      "odom", "base_link", std::numeric_limits<double>::quiet_NaN()), std::nullopt);
  const auto argument_log = testing::internal::GetCapturedStderr();
  EXPECT_NE(argument_log.find("must be finite"), std::string::npos);

  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer.transformStampedPose("odom", makePose(9), tf2::Duration::zero()), std::nullopt);
  const auto pose_log = testing::internal::GetCapturedStderr();
  EXPECT_NE(pose_log.find("Failed to get transform from 'base_link' to 'odom'"), std::string::npos);

  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer.getStampedTransform(
      "base_link", now_, "base_link", now_, "missing", tf2::Duration::zero()), std::nullopt);
  const auto fixed_frame_log = testing::internal::GetCapturedStderr();
  EXPECT_NE(fixed_frame_log.find("via 'missing'"), std::string::npos);
}

TEST_F(TransformBufferTest, DisabledLoggingSuppressesNestedLookupsAndValidationFailures)
{
  addTransform(8, 1.0);
  buffer_.setUsingDedicatedThread(true);
  const auto no_wait = tf2::Duration::zero();
  const auto negative_wait = tf2::durationFromSec(-0.1);
  testing::internal::CaptureStderr();
  // Repeat the probes as a caller waiting for TF would do.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(buffer_.getLatestTransform(
        "odom", "missing", 0.0, nav2::Log::Disabled), std::nullopt);
    EXPECT_EQ(buffer_.getLatestPose(
        "odom", "base_link", 1.0, nav2::Log::Disabled), std::nullopt);
    EXPECT_EQ(buffer_.getStampedTransform(
        "odom", "missing", now_, no_wait, nav2::Log::Disabled), std::nullopt);
    EXPECT_EQ(buffer_.getStampedTransform(
        "base_link", now_, "base_link", now_, "missing", no_wait, nav2::Log::Disabled),
      std::nullopt);
    EXPECT_EQ(buffer_.transformStampedPose(
        "odom", makePose(9), no_wait, nav2::Log::Disabled), std::nullopt);
  }
  EXPECT_EQ(buffer_.getLatestTransform(
      "odom", "base_link", 1.0, nav2::Log::Disabled), std::nullopt);
  nav2::TransformBuffer timed_buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  timed_buffer.setUsingDedicatedThread(true);
  EXPECT_EQ(timed_buffer.getStampedTransform(
      "odom", "missing", now_, tf2::durationFromSec(0.001), nav2::Log::Disabled), std::nullopt);
  EXPECT_EQ(buffer_.getLatestTransform(
      "odom", "base_link", std::numeric_limits<double>::quiet_NaN(), nav2::Log::Disabled),
    std::nullopt);
  EXPECT_EQ(buffer_.getStampedTransform(
      "base_link", "base_link", now_, negative_wait, nav2::Log::Disabled), std::nullopt);
  EXPECT_EQ(buffer_.getStampedTransform(
      "base_link", now_, "base_link", now_, "odom", negative_wait, nav2::Log::Disabled),
    std::nullopt);
  EXPECT_EQ(buffer_.transformStampedPose(
      "base_link", makePose(8), negative_wait, nav2::Log::Disabled), std::nullopt);
  EXPECT_EQ(testing::internal::GetCapturedStderr(), "");

  // A quiet probe must not change the shared buffer's subsequent logging policy.
  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer_.getLatestPose("odom", "missing"), std::nullopt);
  EXPECT_NE(testing::internal::GetCapturedStderr().find("missing"), std::string::npos);
}

TEST_F(TransformBufferTest, LoggingOptionPreservesSuccessfulResults)
{
  addTransform(8, 1.0);
  const auto no_wait = tf2::Duration::zero();
  EXPECT_EQ(buffer_.getLatestTransform("odom", "base_link", 2.0, nav2::Log::Disabled),
    buffer_.getLatestTransform("odom", "base_link", 2.0, nav2::Log::Enabled));
  EXPECT_EQ(buffer_.getLatestPose("odom", "base_link", 2.0, nav2::Log::Disabled),
    buffer_.getLatestPose("odom", "base_link", 2.0));
  EXPECT_EQ(buffer_.getStampedTransform("odom", "base_link", rclcpp::Time(8, 0), no_wait,
      nav2::Log::Disabled),
    buffer_.getStampedTransform("odom", "base_link", rclcpp::Time(8, 0), no_wait));
  EXPECT_EQ(buffer_.transformStampedPose("odom", makePose(8), no_wait, nav2::Log::Disabled),
    buffer_.transformStampedPose("odom", makePose(8), no_wait));
}
