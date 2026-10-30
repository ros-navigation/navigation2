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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rcl/time.h"
#include "nav2_ros_common/tf_manager.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"

class TransformManagerTest : public ::testing::Test
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
    ASSERT_TRUE(buffer_.getRawBuffer().setTransform(transform, "test", is_static));
  }

  geometry_msgs::msg::PoseStamped makePose(int seconds) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = rclcpp::Time(seconds, 0, RCL_ROS_TIME);
    pose.pose.position.x = 1.0;
    return pose;
  }

  nav2::LifecycleNode::SharedPtr node_ = std::make_shared<nav2::LifecycleNode>("tf_manager_test");
  rclcpp::Clock::SharedPtr clock_ = node_->get_clock();
  nav2::TransformManager buffer_{node_, false};
  const rclcpp::Time now_{10, 0, RCL_ROS_TIME};
};

TEST_F(TransformManagerTest, LatestStalenessAndDisabledCheck)
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

TEST_F(TransformManagerTest, QuietLookupsRespectTimestamp)
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

TEST_F(TransformManagerTest, PoseTransformConversionsPreserveHeaderAndGeometry)
{
  addTransform(9, 1.0);
  auto transform = buffer_.getLatestTransform("odom", "base_link");
  ASSERT_TRUE(transform.has_value());

  const auto pose = buffer_.transformToPoseStamped(*transform);
  EXPECT_EQ(pose.header, transform->header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 3.0);
  EXPECT_EQ(pose.pose.orientation, transform->transform.rotation);
  EXPECT_EQ(
    buffer_.poseToTransformStamped(pose, transform->child_frame_id), *transform);
  EXPECT_EQ(
    buffer_.poseToTransformStamped(pose, "other_child").child_frame_id,
    "other_child");
}

TEST_F(TransformManagerTest, LatestStaticAndFutureTransforms)
{
  addTransform(1, 1.0, true);
  auto result = buffer_.getLatestTransform("odom", "base_link", 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 0);
  EXPECT_EQ(result->header.stamp.nanosec, 0u);

  buffer_.getRawBuffer().clear();
  addTransform(11, 2.0);
  result = buffer_.getLatestTransform("odom", "base_link", 0.1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.stamp.sec, 11);
}

TEST_F(TransformManagerTest, SameFrameIdentity)
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

TEST_F(TransformManagerTest, LatestPosePreservesHeaderAndGeometry)
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

TEST_F(TransformManagerTest, StampedLookupInterpolatesHistoricalTransform)
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

TEST_F(TransformManagerTest, ZeroStampLooksUpLatestTransform)
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

TEST_F(TransformManagerTest, DifferentTimesInSameFrameAreNotIdentity)
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

TEST_F(TransformManagerTest, ZeroSourceAndTargetTimesLookUpLatestTransforms)
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

TEST_F(TransformManagerTest, GetStampedPoseInterpolatesAndPreservesInput)
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

TEST_F(TransformManagerTest, GetStampedPoseZeroStampSelectsLatestWithoutAgeCheck)
{
  addTransform(2, 2.0);
  addTransform(4, 4.0);
  const auto result = buffer_.transformStampedPose("odom", makePose(0), tf2::Duration::zero());
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp.sec, 4);
  EXPECT_NEAR(result->pose.position.x, 4.28, 1e-12);
}

TEST_F(TransformManagerTest, TransformPoseSameFramePreservesEntireInputWithoutTF)
{
  const auto input = makePose(2);
  const auto stamped = buffer_.transformStampedPose("base_link", input, tf2::Duration::zero());
  ASSERT_TRUE(stamped);
  EXPECT_EQ(*stamped, input);
}

TEST_F(TransformManagerTest, TransformPoseFailuresLeaveInputUnchanged)
{
  addTransform(2, 2.0);
  const auto input = makePose(3);
  const auto original = input;
  EXPECT_EQ(buffer_.transformStampedPose("odom", input, tf2::Duration::zero()), std::nullopt);
  EXPECT_EQ(buffer_.transformStampedPose("missing", input, tf2::Duration::zero()), std::nullopt);
  EXPECT_EQ(input, original);
}

TEST_F(TransformManagerTest, LatestLookupsUseStoredClock)
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

TEST_F(TransformManagerTest, StalenessTracksClockAdvancement)
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

TEST_F(TransformManagerTest, RejectsNullNode)
{
  EXPECT_THROW(nav2::TransformManager(nav2::LifecycleNode::SharedPtr{}), std::invalid_argument);
}

TEST_F(TransformManagerTest, OwnedBufferUsesProvidedClockForStaleness)
{
  nav2::TransformManager owned(node_, false);
  ASSERT_TRUE(owned.getRawBuffer().setTransform(
      owned.poseToTransformStamped(makePose(9), "sensor"), "test"));
  EXPECT_TRUE(owned.getLatestTransform("base_link", "sensor", 1.0));
  setTime(rclcpp::Time(11, 0, RCL_ROS_TIME));
  EXPECT_EQ(owned.getLatestTransform("base_link", "sensor", 1.0), std::nullopt);
}

TEST_F(TransformManagerTest, RawBufferAccessReferencesOwnedBuffer)
{
  addTransform(9, 1.0);
  const auto & const_buffer = buffer_;
  EXPECT_EQ(&const_buffer.getRawBuffer(), &buffer_.getRawBuffer());
  EXPECT_EQ(const_buffer.getRawBuffer().lookupTransform("odom", "base_link", tf2::TimePointZero),
    *buffer_.getLatestTransform("odom", "base_link"));
}

TEST_F(TransformManagerTest, InversePreservesTimestampAndExchangesFrames)
{
  addTransform(9, 1.0);
  const auto transform = buffer_.getLatestTransform("odom", "base_link");
  ASSERT_TRUE(transform);
  const auto inverse = buffer_.invertTransform(*transform);
  EXPECT_EQ(inverse.header.frame_id, transform->child_frame_id);
  EXPECT_EQ(inverse.child_frame_id, transform->header.frame_id);
  EXPECT_EQ(inverse.header.stamp, transform->header.stamp);
  const auto restored = buffer_.invertTransform(inverse);
  EXPECT_NEAR(restored.transform.translation.x, transform->transform.translation.x, 1e-12);
  EXPECT_NEAR(restored.transform.translation.y, transform->transform.translation.y, 1e-12);
  EXPECT_NEAR(restored.transform.translation.z, transform->transform.translation.z, 1e-12);
  EXPECT_NEAR(restored.transform.rotation.z, transform->transform.rotation.z, 1e-12);
  EXPECT_NEAR(restored.transform.rotation.w, transform->transform.rotation.w, 1e-12);
}

TEST_F(TransformManagerTest, RejectsNegativeToleranceIncludingIdentity)
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

TEST_F(TransformManagerTest, RejectsNonfiniteStalenessThreshold)
{
  addTransform(9, 1.0);
  EXPECT_EQ(buffer_.getLatestTransform(
      "odom", "base_link", std::numeric_limits<double>::quiet_NaN()), std::nullopt);
  EXPECT_EQ(buffer_.getLatestPose(
      "odom", "base_link", std::numeric_limits<double>::infinity()), std::nullopt);
}

TEST_F(TransformManagerTest, LogsFailuresWithConfiguredLoggerAndFrameContext)
{
  nav2::TransformManager configured_buffer(node_, false);
  const auto & buffer = configured_buffer;
  testing::internal::CaptureStderr();
  EXPECT_EQ(buffer.getLatestPose("odom", "missing"), std::nullopt);
  const auto lookup_log = testing::internal::GetCapturedStderr();
  EXPECT_NE(lookup_log.find("tf_manager_test"), std::string::npos);
  EXPECT_NE(lookup_log.find("Failed to get transform from 'missing' to 'odom'"), std::string::npos);

  addTransform(8, 1.0);
  ASSERT_TRUE(configured_buffer.getRawBuffer().setTransform(
      *buffer_.getLatestTransform("odom", "base_link"), "test"));
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

TEST_F(TransformManagerTest, DisabledLoggingSuppressesNestedLookupsAndValidationFailures)
{
  addTransform(8, 1.0);
  buffer_.getRawBuffer().setUsingDedicatedThread(true);
  const auto no_wait = tf2::Duration::zero();
  const auto negative_wait = tf2::durationFromSec(-0.1);
  auto system_node = std::make_shared<nav2::LifecycleNode>(
    "tf_manager_system_time", "", rclcpp::NodeOptions().clock_type(RCL_SYSTEM_TIME));
  nav2::TransformManager timed_buffer(system_node, false);
  timed_buffer.getRawBuffer().setUsingDedicatedThread(true);
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

TEST_F(TransformManagerTest, LoggingOptionPreservesSuccessfulResults)
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

class TransformListenerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("transform_listener_test");
    executor_.add_node(node_->get_node_base_interface());
    buffer_ = nav2::create_transform_manager(node_, false);
  }

  void expectTransforms()
  {
    geometry_msgs::msg::TransformStamped dynamic_transform;
    dynamic_transform.header.frame_id = "listener_parent";
    dynamic_transform.child_frame_id = "listener_dynamic";
    dynamic_transform.header.stamp = node_->now();
    dynamic_transform.transform.translation.x = 2.0;
    auto static_transform = dynamic_transform;
    static_transform.child_frame_id = "listener_static";
    static_transform.transform.translation.x = 3.0;
    buffer_->sendStaticTransform(static_transform);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      buffer_->sendTransform(dynamic_transform);
      executor_.spin_some();
      const auto dynamic_result = buffer_->getLatestTransform(
        "listener_parent", "listener_dynamic", 0.0, nav2::Log::Disabled);
      const auto static_result = buffer_->getLatestTransform(
        "listener_parent", "listener_static", 0.0, nav2::Log::Disabled);
      if (dynamic_result && static_result) {
        EXPECT_DOUBLE_EQ(dynamic_result->transform.translation.x, 2.0);
        EXPECT_DOUBLE_EQ(static_result->transform.translation.x, 3.0);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FAIL() << "Owned listener did not receive dynamic and static TF";
  }

  void expectStaticTransforms(
    const nav2::TransformManager & receiver,
    const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      bool all_received = true;
      for (const auto & transform : transforms) {
        const auto result = receiver.getLatestTransform(
          transform.header.frame_id, transform.child_frame_id, 0.001, nav2::Log::Disabled);
        if (!result) {
          all_received = false;
          break;
        }
        EXPECT_DOUBLE_EQ(result->transform.translation.x, transform.transform.translation.x);
        EXPECT_EQ(rclcpp::Time(result->header.stamp).nanoseconds(), 0);
      }
      if (all_received) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FAIL() << "Manager did not receive all retained static transforms";
  }

  nav2::LifecycleNode::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  nav2::TransformManager::SharedPtr buffer_;
};

TEST_F(TransformListenerTest, ListenerReceivesTransformsByDefault)
{
  buffer_ = nav2::create_transform_manager(node_);
  EXPECT_TRUE(buffer_->getRawBuffer().isUsingDedicatedThread());
  expectTransforms();

  // Destruction must join the listener thread while the native buffer is still alive.
  buffer_.reset();
}

TEST_F(TransformListenerTest, OptionalListenerCanBeStoppedAndRestarted)
{
  EXPECT_FALSE(buffer_->getRawBuffer().isUsingDedicatedThread());
  buffer_->stopListener();
  buffer_->startListener();
  EXPECT_TRUE(buffer_->getRawBuffer().isUsingDedicatedThread());
  buffer_->startListener();
  expectTransforms();

  buffer_->stopListener();
  EXPECT_FALSE(buffer_->getRawBuffer().isUsingDedicatedThread());
  EXPECT_TRUE(buffer_->getLatestTransform("listener_parent", "listener_dynamic"));
  buffer_->stopListener();
  buffer_->getRawBuffer().clear();
  buffer_->startListener();
  expectTransforms();
}

TEST_F(TransformListenerTest, CallerCanSpinListener)
{
  buffer_ = nav2::create_transform_manager(node_, false, false);
  buffer_->startListener();
  EXPECT_FALSE(buffer_->getRawBuffer().isUsingDedicatedThread());
  expectTransforms();
  buffer_->stopListener();
  buffer_->startListener();
  expectTransforms();
}

TEST_F(TransformListenerTest, BroadcasterIsAdvertisedOnFirstSend)
{
  const auto publishers = node_->count_publishers("/tf");
  const auto static_publishers = node_->count_publishers("/tf_static");
  buffer_ = nav2::create_transform_manager(node_, false);
  EXPECT_EQ(node_->count_publishers("/tf"), publishers);
  EXPECT_EQ(node_->count_publishers("/tf_static"), static_publishers);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "broadcast_parent";
  transform.child_frame_id = "broadcast_child";
  buffer_->sendTransform(transform);
  EXPECT_EQ(node_->count_publishers("/tf"), publishers + 1);
  buffer_->sendTransform(transform);
  EXPECT_EQ(node_->count_publishers("/tf"), publishers + 1);
  EXPECT_EQ(node_->count_publishers("/tf_static"), static_publishers);
}

TEST_F(TransformListenerTest, StaticBroadcasterIsAdvertisedOnFirstSend)
{
  const auto publishers = node_->count_publishers("/tf");
  const auto static_publishers = node_->count_publishers("/tf_static");
  buffer_ = nav2::create_transform_manager(node_, false);
  EXPECT_EQ(node_->count_publishers("/tf"), publishers);
  EXPECT_EQ(node_->count_publishers("/tf_static"), static_publishers);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "static_parent";
  transform.child_frame_id = "static_child";
  buffer_->sendStaticTransform(transform);
  EXPECT_EQ(node_->count_publishers("/tf_static"), static_publishers + 1);
  buffer_->sendStaticTransform(transform);
  EXPECT_EQ(node_->count_publishers("/tf_static"), static_publishers + 1);
  EXPECT_EQ(node_->count_publishers("/tf"), publishers);
}

TEST_F(TransformListenerTest, StaticTransformsAreRetainedForLateListeners)
{
  auto sender = nav2::create_transform_manager(node_, false);
  std::vector<geometry_msgs::msg::TransformStamped> transforms(3);
  for (size_t i = 0; i < transforms.size(); ++i) {
    transforms[i].header.frame_id = "static_parent";
    transforms[i].child_frame_id = "static_child_" + std::to_string(i);
    transforms[i].header.stamp = node_->now();
    transforms[i].transform.translation.x = static_cast<double>(i + 1);
  }
  sender->sendStaticTransform(transforms[0]);
  sender->sendStaticTransform(
    std::vector<geometry_msgs::msg::TransformStamped>{transforms[1], transforms[2]});

  auto late_node = std::make_shared<nav2::LifecycleNode>("late_static_listener");
  auto receiver = nav2::create_transform_manager(late_node);
  expectStaticTransforms(*receiver, transforms);
}

TEST_F(TransformListenerTest, ConcurrentStaticSendsRetainEveryTransform)
{
  const auto static_publishers = node_->count_publishers("/tf_static");
  auto sender = nav2::create_transform_manager(node_, false);
  std::vector<geometry_msgs::msg::TransformStamped> transforms(8);
  for (size_t i = 0; i < transforms.size(); ++i) {
    transforms[i].header.frame_id = "concurrent_static_parent";
    transforms[i].child_frame_id = "concurrent_static_child_" + std::to_string(i);
    transforms[i].transform.translation.x = static_cast<double>(i + 1);
  }
  std::vector<std::thread> threads;
  for (const auto & transform : transforms) {
    threads.emplace_back([sender, transform]() {sender->sendStaticTransform(transform);});
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(node_->count_publishers("/tf_static"), static_publishers + 1);
  auto late_node = std::make_shared<nav2::LifecycleNode>("concurrent_static_listener");
  auto receiver = nav2::create_transform_manager(late_node);
  expectStaticTransforms(*receiver, transforms);
}

TEST_F(TransformListenerTest, BroadcastsTransformVector)
{
  buffer_->startListener();
  geometry_msgs::msg::TransformStamped first;
  first.header.frame_id = "broadcast_parent";
  first.child_frame_id = "broadcast_first";
  first.header.stamp = node_->now();
  first.transform.translation.x = 2.0;
  auto second = first;
  second.child_frame_id = "broadcast_second";
  second.transform.translation.x = 3.0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    buffer_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{first, second});
    const auto first_result = buffer_->getLatestTransform(
      "broadcast_parent", "broadcast_first", 0.0, nav2::Log::Disabled);
    const auto second_result = buffer_->getLatestTransform(
      "broadcast_parent", "broadcast_second", 0.0, nav2::Log::Disabled);
    if (first_result && second_result) {
      EXPECT_DOUBLE_EQ(first_result->transform.translation.x, 2.0);
      EXPECT_DOUBLE_EQ(second_result->transform.translation.x, 3.0);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL() << "Manager did not broadcast both transforms";
}

TEST_F(TransformListenerTest, ManagerDoesNotRetainOwningNode)
{
  auto node = std::make_shared<nav2::LifecycleNode>("tf_manager_lifetime");
  const std::weak_ptr<nav2::LifecycleNode> weak_node = node;
  auto manager = nav2::create_transform_manager(node, false);
  node.reset();
  EXPECT_TRUE(weak_node.expired());
  manager.reset();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
