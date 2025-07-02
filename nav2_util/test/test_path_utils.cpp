// Copyright (c) 2024, Berkan Tali
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

#include "gtest/gtest.h"
#include <limits>
#include <cmath>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/path_utils.hpp" // Assuming this is the correct path

// --- TESTS FOR GLOBAL SEARCH (closest_idx = nullptr) ---

TEST(DistanceFromPath, EmptyPath)
{
  geometry_msgs::msg::PoseStamped pose;
  nav_msgs::msg::Path path;
  EXPECT_EQ(nav2_util::distanceFromPath(pose, path, nullptr),
    std::numeric_limits<double>::infinity());
}

TEST(DistanceFromPath, SinglePointPath)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped path_pose;
  path_pose.pose.position.x = 3.0;
  path_pose.pose.position.y = 4.0;
  path.poses.push_back(path_pose);

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}

TEST(DistanceFromPath, PerpendicularToSegment)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 5.0;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  path.poses.push_back(p1);
  path.poses.push_back(p2);

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}

TEST(DistanceFromPath, ClosestToVertex)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = -5.0;
  pose.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  path.poses.push_back(p1);
  path.poses.push_back(p2);

  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}


// --- TESTS FOR ITERATIVE SEARCH (using closest_idx) ---

TEST(DistanceFromPath, IterativeSearch)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 15.0;
  pose.pose.position.y = 5.0;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  p3.pose.position.x = 20.0;
  path.poses.push_back(p1);
  path.poses.push_back(p2);
  path.poses.push_back(p3);

  size_t search_start_index = 0;
  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, &search_start_index), 5.0, 1e-6);
  // After the call, the index should be updated to point to the start of the closest segment (p2->p3)
  EXPECT_EQ(search_start_index, 1u);
}

TEST(DistanceFromPath, IterativeSearchStartsFromGivenIndex)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = -5.0; // This point is closer to the first segment

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  p3.pose.position.x = 20.0;
  path.poses.push_back(p1);
  path.poses.push_back(p2);
  path.poses.push_back(p3);

  // Start the search from index 1, ignoring the first segment (p1->p2)
  size_t search_start_index = 1;
  // The closest point should now be p2 (10,0), not the point (5,-5) projected onto p1->p2
  double expected_dist = std::sqrt(std::pow(10 - 5, 2) + std::pow(0 - (-5), 2)); // sqrt(5^2 + 5^2) = sqrt(50)
  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, &search_start_index), expected_dist, 1e-6);
}

// --- TEST FOR CLOSED PATHS ---

TEST(DistanceFromPath, ClosedPathSeam)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 15.0;
  pose.pose.position.y = 5.0;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1, p2, p3;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 10.0;
  p3.pose.position.x = 20.0;
  p3.pose.position.y = 10.0;
  path.poses.push_back(p1);
  path.poses.push_back(p2);
  path.poses.push_back(p3);
  // Add the first point again to make it a closed loop
  path.poses.push_back(p1);

  // The pose (15, 5) is closer to the virtual "seam" segment (p3->p1)
  // than it is to the p1->p2 or p2->p3 segments.
  EXPECT_NEAR(nav2_util::distanceFromPath(pose, path, nullptr), 5.0, 1e-6);
}
