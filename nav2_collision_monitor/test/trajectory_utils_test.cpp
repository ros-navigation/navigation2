// Copyright (c) 2026 Polymath Robotics
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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav_msgs/msg/trajectory.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"

#include "nav2_collision_monitor/trajectory_utils.hpp"
#include "nav2_collision_monitor/types.hpp"

using nav2_collision_monitor::TimedPose;
using namespace nav2_collision_monitor::trajectory_utils;  // NOLINT

static constexpr double EPSILON = 1e-6;

namespace
{

builtin_interfaces::msg::Time stamp(double seconds)
{
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(std::floor(seconds));
  msg.nanosec = static_cast<uint32_t>(std::round((seconds - msg.sec) * 1e9));
  return msg;
}

nav_msgs::msg::TrajectoryPoint makePoint(double seconds, double x, double y, double yaw = 0.0)
{
  nav_msgs::msg::TrajectoryPoint point;
  point.header.stamp = stamp(seconds);
  point.header.frame_id = "odom";
  point.pose.position.x = x;
  point.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  point.pose.orientation.x = q.x();
  point.pose.orientation.y = q.y();
  point.pose.orientation.z = q.z();
  point.pose.orientation.w = q.w();
  return point;
}

// Straight line along +x: stamp 100 + i * 0.1, x = i * 0.1
nav_msgs::msg::Trajectory makeLine(std::size_t size)
{
  nav_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = stamp(100.0);
  trajectory.header.frame_id = "odom";
  for (std::size_t i = 0; i < size; i++) {
    const double t = static_cast<double>(i) * 0.1;
    trajectory.points.push_back(makePoint(100.0 + t, t, 0.0));
  }
  return trajectory;
}

}  // namespace

TEST(TrajectoryUtilsTest, validateAcceptsLine)
{
  std::string reason;
  EXPECT_TRUE(validate(makeLine(5), true, reason));
}

TEST(TrajectoryUtilsTest, validateRejectsEmpty)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory;
  EXPECT_FALSE(validate(trajectory, false, reason));
  EXPECT_FALSE(reason.empty());
}

TEST(TrajectoryUtilsTest, validateRejectsNonFinite)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory = makeLine(3);
  trajectory.points[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validate(trajectory, false, reason));

  trajectory = makeLine(3);
  trajectory.points[1].pose.orientation = geometry_msgs::msg::Quaternion();
  trajectory.points[1].pose.orientation.w = 0.0;
  EXPECT_FALSE(validate(trajectory, false, reason));
}

TEST(TrajectoryUtilsTest, validateRejectsZeroStamp)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory = makeLine(3);
  trajectory.points[2].header.stamp = builtin_interfaces::msg::Time();
  EXPECT_FALSE(validate(trajectory, false, reason));
}

TEST(TrajectoryUtilsTest, validateRejectsNonMonotonicStamps)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory = makeLine(3);
  trajectory.points[2].header.stamp = stamp(100.05);
  EXPECT_FALSE(validate(trajectory, false, reason));
}

TEST(TrajectoryUtilsTest, validateRejectsMixedFrames)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory = makeLine(3);
  trajectory.points[2].header.frame_id = "map";
  EXPECT_FALSE(validate(trajectory, false, reason));
}

TEST(TrajectoryUtilsTest, validateInheritsHeaderFrame)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory = makeLine(3);
  for (auto & point : trajectory.points) {
    point.header.frame_id.clear();
  }
  EXPECT_TRUE(validate(trajectory, false, reason));
  EXPECT_EQ("odom", getFrameId(trajectory));

  trajectory.header.frame_id.clear();
  EXPECT_FALSE(validate(trajectory, false, reason));
}

TEST(TrajectoryUtilsTest, validateTimeSpan)
{
  std::string reason;
  nav_msgs::msg::Trajectory trajectory = makeLine(3);
  for (auto & point : trajectory.points) {
    point.header.stamp = stamp(100.0);
  }
  EXPECT_TRUE(validate(trajectory, false, reason));
  EXPECT_FALSE(validate(trajectory, true, reason));
  // A single point has no span to require
  EXPECT_TRUE(validate(makeLine(1), true, reason));
}

TEST(TrajectoryUtilsTest, toBasePosesTransformsPoses)
{
  // Trajectory frame is the base frame rotated by 90 degrees and shifted to (1, 2)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI / 2.0);
  const tf2::Transform to_base(q, tf2::Vector3(1.0, 2.0, 0.0));

  nav_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "odom";
  trajectory.points.push_back(makePoint(100.0, 1.0, 0.0, 0.0));
  const auto poses = toBasePoses(trajectory, to_base, rclcpp::Time(100, 0), false);
  ASSERT_EQ(1u, poses.size());
  EXPECT_NEAR(1.0, poses[0].pose.x, EPSILON);
  EXPECT_NEAR(3.0, poses[0].pose.y, EPSILON);
  EXPECT_NEAR(M_PI / 2.0, poses[0].pose.theta, EPSILON);
}

TEST(TrajectoryUtilsTest, toBasePosesDropsPastPointsAndInsertsLabelZero)
{
  const auto trajectory = makeLine(5);  // stamps 100.0 .. 100.4
  const auto poses = toBasePoses(
    trajectory, tf2::Transform::getIdentity(), rclcpp::Time(100, 150000000), true);
  // Points at 100.0 and 100.1 are dropped; a pose is interpolated at now = 100.15
  ASSERT_EQ(4u, poses.size());
  EXPECT_NEAR(0.0, poses[0].time_s, EPSILON);
  EXPECT_NEAR(0.15, poses[0].pose.x, EPSILON);
  EXPECT_NEAR(0.05, poses[1].time_s, EPSILON);
  EXPECT_NEAR(0.2, poses[1].pose.x, EPSILON);
}

TEST(TrajectoryUtilsTest, toBasePosesKeepsPointAtNow)
{
  const auto trajectory = makeLine(5);
  const auto poses = toBasePoses(
    trajectory, tf2::Transform::getIdentity(), rclcpp::Time(100, 200000000), true);
  ASSERT_EQ(3u, poses.size());
  EXPECT_NEAR(0.0, poses[0].time_s, EPSILON);
  EXPECT_NEAR(0.2, poses[0].pose.x, EPSILON);
}

TEST(TrajectoryUtilsTest, toBasePosesAllInThePastIsEmpty)
{
  const auto trajectory = makeLine(5);
  EXPECT_TRUE(
    toBasePoses(trajectory, tf2::Transform::getIdentity(), rclcpp::Time(200, 0), true).empty());
}

TEST(TrajectoryUtilsTest, toBasePosesFirstStampIsTimeZeroWithoutTrajectoryTime)
{
  const auto trajectory = makeLine(5);
  for (const int64_t now_s : {0, 100, 200}) {
    const auto poses = toBasePoses(
      trajectory, tf2::Transform::getIdentity(), rclcpp::Time(now_s, 0), false);
    ASSERT_EQ(5u, poses.size());
    EXPECT_NEAR(0.0, poses[0].time_s, EPSILON);
    EXPECT_NEAR(0.1, poses[1].time_s, EPSILON);
    EXPECT_NEAR(0.4, poses[4].time_s, EPSILON);
  }
}

TEST(TrajectoryUtilsTest, interpolatePassesThroughWithoutStep)
{
  const std::vector<TimedPose> poses{{0.0, {0, 0, 0}}, {0.5, {1, 0, 0}}, {1.5, {3, 0, 0}}};
  EXPECT_EQ(3u, interpolate(poses, 10.0, 0.0).size());
  EXPECT_EQ(3u, interpolate(poses, 10.0, -1.0).size());
  // Horizon is inclusive
  EXPECT_EQ(2u, interpolate(poses, 0.5, 0.0).size());
  EXPECT_EQ(1u, interpolate(poses, 0.49, 0.0).size());
  EXPECT_TRUE(interpolate(poses, -0.1, 0.0).empty());
}

TEST(TrajectoryUtilsTest, interpolateResamplesCoarsePoses)
{
  const std::vector<TimedPose> poses{{0.0, {0, 0, 0}}, {1.0, {2, 0, 0}}};
  const auto sampled = interpolate(poses, 10.0, 0.25);
  ASSERT_EQ(5u, sampled.size());
  EXPECT_NEAR(0.0, sampled[0].time_s, EPSILON);
  EXPECT_NEAR(0.5, sampled[1].pose.x, EPSILON);
  EXPECT_NEAR(0.25, sampled[1].time_s, EPSILON);
  EXPECT_NEAR(2.0, sampled[4].pose.x, EPSILON);
  EXPECT_NEAR(1.0, sampled[4].time_s, EPSILON);
}

TEST(TrajectoryUtilsTest, interpolateEndsAtTrajectoryEndOffGrid)
{
  const std::vector<TimedPose> poses{{0.0, {0, 0, 0}}, {1.0, {1, 0, 0}}};
  const auto sampled = interpolate(poses, 10.0, 0.3);
  // 0.0, 0.3, 0.6, 0.9, then the end at 1.0
  ASSERT_EQ(5u, sampled.size());
  EXPECT_NEAR(1.0, sampled.back().time_s, EPSILON);
  EXPECT_NEAR(1.0, sampled.back().pose.x, EPSILON);
}

TEST(TrajectoryUtilsTest, interpolateCutsAtHorizon)
{
  const std::vector<TimedPose> poses{{0.0, {0, 0, 0}}, {4.0, {4, 0, 0}}};
  const auto sampled = interpolate(poses, 2.0, 0.5);
  ASSERT_EQ(5u, sampled.size());
  EXPECT_NEAR(2.0, sampled.back().time_s, EPSILON);
  EXPECT_NEAR(2.0, sampled.back().pose.x, EPSILON);
}

TEST(TrajectoryUtilsTest, interpolateStartsAtFirstLabel)
{
  const std::vector<TimedPose> poses{{0.3, {0, 0, 0}}, {1.3, {1, 0, 0}}};
  const auto sampled = interpolate(poses, 10.0, 0.5);
  EXPECT_NEAR(0.3, sampled.front().time_s, EPSILON);
  EXPECT_TRUE(interpolate(poses, 0.2, 0.5).empty());
}

TEST(TrajectoryUtilsTest, interpolateSinglePoseIsUnchanged)
{
  const std::vector<TimedPose> poses{{0.2, {1, 2, 0.5}}};
  const auto sampled = interpolate(poses, 2.0, 0.1);
  ASSERT_EQ(1u, sampled.size());
  EXPECT_NEAR(0.2, sampled[0].time_s, EPSILON);
  EXPECT_NEAR(1.0, sampled[0].pose.x, EPSILON);
  EXPECT_TRUE(interpolate(poses, 0.1, 0.1).empty());
}

TEST(TrajectoryUtilsTest, interpolateTakesShortestYawArc)
{
  // From 170 deg to -170 deg is a 20 degree turn through 180, not 340 through 0
  const double from = 170.0 * M_PI / 180.0;
  const double to = -170.0 * M_PI / 180.0;
  const std::vector<TimedPose> poses{{0.0, {0, 0, from}}, {1.0, {0, 0, to}}};
  const auto sampled = interpolate(poses, 1.0, 0.5);
  ASSERT_EQ(3u, sampled.size());
  EXPECT_NEAR(M_PI, std::abs(sampled[1].pose.theta), EPSILON);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
