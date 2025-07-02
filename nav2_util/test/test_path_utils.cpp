/******************************************************************************
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#include <cstddef>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/path_utils.hpp"

// Common frame id used in all tests (poses and paths must match)
static constexpr const char * kFrame = "map";

// ---------------------------------------------------------------------------
//  GLOBAL-SEARCH TESTS  (closest_idx == nullptr)
// ---------------------------------------------------------------------------

TEST(DistanceFromPath, EmptyPath)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;

  EXPECT_EQ(nav2_util::distanceFromPath(pose, path, nullptr),
            std::numeric_limits<double>::infinity());
}

TEST(DistanceFromPath, SinglePointPath)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 3.0;
  p.pose.position.y = 4.0;
  path.poses.push_back(p);

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}

TEST(DistanceFromPath, PerpendicularToSegment)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 5.0;

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  path.poses = {p1, p2};

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}

TEST(DistanceFromPath, ClosestToVertex)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;
  pose.pose.position.x = -5.0;
  pose.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  path.poses = {p1, p2};

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}

// ---------------------------------------------------------------------------
//  ITERATIVE (LOCAL) SEARCH TESTS  (closest_idx != nullptr)
// ---------------------------------------------------------------------------

TEST(DistanceFromPath, IterativeSearchUpdatesIndex)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;
  pose.pose.position.x = 15.0;
  pose.pose.position.y = 5.0;

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;
  geometry_msgs::msg::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  p3.pose.position.x = 20.0;
  path.poses = {p1, p2, p3};

  size_t idx = 0;
  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, &idx), 5.0, 1e-6);
  EXPECT_EQ(idx, 1u);  // p2->p3 segment
}

TEST(DistanceFromPath, IterativeSearchRespectsStartIndex)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = -5.0;   // Closer to p1->p2 but we’ll skip it

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;
  geometry_msgs::msg::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  p3.pose.position.x = 20.0;
  path.poses = {p1, p2, p3};

  size_t idx = 1;  // start search at p2->p3
  const double expected = std::sqrt(50.0);  // distance to (10,0)
  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, &idx), expected, 1e-6);
  EXPECT_EQ(idx, 1u);
}

// ---------------------------------------------------------------------------
//  CLOSED-LOOP TEST
// ---------------------------------------------------------------------------

TEST(DistanceFromPath, ClosedLoopSeam)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kFrame;
  pose.pose.position.x = 15.0;
  pose.pose.position.y = 5.0;

  nav_msgs::msg::Path path;
  path.header.frame_id = kFrame;
  geometry_msgs::msg::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  p3.pose.position.x = 20.0;  p3.pose.position.y = 10.0;
  path.poses = {p1, p2, p3, p1};         // closed

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}
