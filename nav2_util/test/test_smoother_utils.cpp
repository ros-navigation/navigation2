// Copyright (c) 2025 Maurice Alexander Purnawan
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
#include "nav2_util/smoother_utils.hpp"
#include "gtest/gtest.h"

using nav2_util::findDirectionalPathSegments;
using nav2_util::updateApproximatePathOrientations;

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return pose;
}

TEST(SmootherUtils, HolonomicSingleSegment)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0, 0));
  path.poses.push_back(makePose(1, 0));
  path.poses.push_back(makePose(2, 0));

  auto segments = findDirectionalPathSegments(path, true);
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_EQ(segments[0].start, 0u);
  EXPECT_EQ(segments[0].end, 2u);
}

TEST(SmootherUtils, ForwardAndReverseSegments)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0, 0));
  path.poses.push_back(makePose(1, 0));
  path.poses.push_back(makePose(0, 0));
  path.poses.push_back(makePose(-1, 0));

  auto segments = findDirectionalPathSegments(path, false);
  ASSERT_GE(segments.size(), 2u);
  EXPECT_EQ(segments[0].start, 0u);
  EXPECT_LT(segments[0].end, path.poses.size());
}

TEST(SmootherUtils, RotationInPlaceCreatesSegment)
{
  nav_msgs::msg::Path path;
  // Same position, but rotating
  path.poses.push_back(makePose(0, 0, 0.0));
  path.poses.push_back(makePose(0, 0, 0.0));
  path.poses.push_back(makePose(0, 0, M_PI_2));
  path.poses.push_back(makePose(0, 0, M_PI));

  auto segments = findDirectionalPathSegments(path, false);
  ASSERT_GE(segments.size(), 2u);
}

TEST(SmootherUtils, UpdateApproximatePathForward)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0, 0, 0.0));
  path.poses.push_back(makePose(1, 0, 0.0));
  path.poses.push_back(makePose(2, 0, 0.0));

  bool reversing = false;
  updateApproximatePathOrientations(path, reversing, false);

  EXPECT_FALSE(reversing);
  double yaw0 = tf2::getYaw(path.poses[0].pose.orientation);
  EXPECT_NEAR(yaw0, 0.0, 1e-5);
}

TEST(SmootherUtils, UpdateApproximatePathReverse)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0, 0, 0));
  path.poses.push_back(makePose(-1, 0, 0));
  path.poses.push_back(makePose(-2, 0, 0));

  bool reversing = false;
  updateApproximatePathOrientations(path, reversing, false);

  EXPECT_TRUE(reversing);
  double yaw0 = tf2::getYaw(path.poses[0].pose.orientation);
  EXPECT_NEAR(yaw0, M_PI, 1e-5);
}
