// Copyright (c) 2025 Berkan Tali
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

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/path_utils.hpp"

geometry_msgs::msg::PoseStamped createPose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "map";
  return pose;
}

void generateCirclePath(
  nav_msgs::msg::Path & path,
  double center_x, double center_y, double radius,
  int num_points, double start_angle = 0.0, double end_angle = 2.0 * M_PI)
{
  const double angle_step = (end_angle - start_angle) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    const double angle = start_angle + i * angle_step;
    path.poses.push_back(createPose(
      center_x + radius * std::cos(angle),
      center_y + radius * std::sin(angle)));
  }
}

class CloverleafPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Target path has three leaves with a radius of 5.0
    generateCirclePath(target_path, 5.0, 0.0, 5.0, 50);
    generateCirclePath(target_path, -5.0, 0.0, 5.0, 50);
    generateCirclePath(target_path, 0.0, 5.0, 5.0, 50);

    // Robot trajectory now also travels all three leaves, but with a radius of 4.8
    nav_msgs::msg::Path robot_path;
    generateCirclePath(robot_path, 5.0, 0.0, 4.8, 50);
    generateCirclePath(robot_path, -5.0, 0.0, 4.8, 50);
    generateCirclePath(robot_path, 0.0, 5.0, 4.8, 50);
    robot_trajectory = robot_path.poses;
  }
  nav_msgs::msg::Path target_path;
  std::vector<geometry_msgs::msg::PoseStamped> robot_trajectory;
};

class RetracingCircleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    generateCirclePath(target_path, 0.0, 0.0, 5.0, 50, 0.0, 2.0 * M_PI);
    generateCirclePath(target_path, 0.0, 0.0, 5.0, 50, 2.0 * M_PI, 0.0);
    nav_msgs::msg::Path robot_path;
    generateCirclePath(robot_path, 0.0, 0.0, 5.2, 50, 0.0, 2.0 * M_PI);
    generateCirclePath(robot_path, 0.0, 0.0, 5.2, 50, 2.0 * M_PI, 0.0);
    robot_trajectory = robot_path.poses;
  }
  nav_msgs::msg::Path target_path;
  std::vector<geometry_msgs::msg::PoseStamped> robot_trajectory;
};

class ZigZagPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_path.poses.push_back(createPose(0.0, 0.0));
    target_path.poses.push_back(createPose(10.0, 0.0));
    target_path.poses.push_back(createPose(10.0, 5.0));
    target_path.poses.push_back(createPose(0.0, 5.0));
    target_path.poses.push_back(createPose(0.0, 10.0));

    robot_trajectory = {
      createPose(1.0, 0.2), createPose(5.0, 0.2), createPose(9.0, 0.2),
      createPose(9.8, 1.0), createPose(10.2, 2.5), createPose(9.8, 4.0),
      createPose(8.0, 5.2), createPose(5.0, 5.2), createPose(2.0, 5.2),
      createPose(0.2, 6.0), createPose(0.2, 8.0), createPose(0.2, 9.8)
    };
  }
  nav_msgs::msg::Path target_path;
  std::vector<geometry_msgs::msg::PoseStamped> robot_trajectory;
};

class HairpinTurnTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_path.poses.push_back(createPose(0.0, 1.0));
    target_path.poses.push_back(createPose(10.0, 1.0));
    target_path.poses.push_back(createPose(10.0, -1.0));
    target_path.poses.push_back(createPose(0.0, -1.0));

    robot_trajectory = {
      createPose(1.0, 1.2), createPose(3.0, 1.2), createPose(5.0, 1.2),
      createPose(7.0, 1.2), createPose(8.5, 1.0), createPose(9.2, 0.5),
      createPose(9.8, 0.0), createPose(9.8, -0.5), createPose(9.2, -1.0),
      createPose(8.0, -1.2), createPose(6.0, -1.2), createPose(4.0, -1.2),
      createPose(2.0, -1.2)
    };
  }
  nav_msgs::msg::Path target_path;
  std::vector<geometry_msgs::msg::PoseStamped> robot_trajectory;
};

class CuttingCornerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_path.poses.push_back(createPose(0.0, 0.0));
    target_path.poses.push_back(createPose(10.0, 0.0));
    target_path.poses.push_back(createPose(10.0, 10.0));

    robot_trajectory = {
      createPose(0.0, 0.2), createPose(2.0, 0.2), createPose(4.0, 0.2),
      createPose(6.0, 0.2), createPose(8.0, 0.2), createPose(9.0, 1.0),
      createPose(9.8, 2.0), createPose(9.8, 4.0), createPose(9.8, 6.0),
      createPose(9.8, 8.0), createPose(9.8, 10.0)
    };
    expected_distances = {0.2, 0.2, 0.2, 0.2, 0.2, 1.0, 0.2, 0.2, 0.2, 0.2, 0.2};
  }
  nav_msgs::msg::Path target_path;
  std::vector<geometry_msgs::msg::PoseStamped> robot_trajectory;
  std::vector<double> expected_distances;
};

class RetracingPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (int i = 0; i <= 10; ++i) {target_path.poses.push_back(createPose(i, 0.0));}
    for (int i = 9; i >= 0; --i) {target_path.poses.push_back(createPose(i, 0.0));}
    for (int i = 0; i <= 10; ++i) {robot_trajectory.push_back(createPose(i, 0.5));}
    for (int i = 9; i >= 0; --i) {robot_trajectory.push_back(createPose(i, 0.5));}
  }
  nav_msgs::msg::Path target_path;
  std::vector<geometry_msgs::msg::PoseStamped> robot_trajectory;
};

TEST(PathUtilsTest, EmptyAndSinglePointPaths)
{
  auto robot_pose = createPose(5.0, 5.0);
  nav_msgs::msg::Path empty_path;

  auto result = nav2_util::distanceFromPath(empty_path, robot_pose);
  EXPECT_EQ(result.distance, std::numeric_limits<double>::infinity());

  nav_msgs::msg::Path single_point_path;
  single_point_path.poses.push_back(createPose(0.0, 0.0));
  result = nav2_util::distanceFromPath(single_point_path, robot_pose);
  EXPECT_NEAR(result.distance, 7.071, 0.01);
}

TEST_F(CuttingCornerTest, TrajectoryCutsCorner)
{
  for (size_t i = 0; i < robot_trajectory.size(); ++i) {
    const auto & robot_pose = robot_trajectory[i];
    auto result = nav2_util::distanceFromPath(target_path, robot_pose);
    EXPECT_NEAR(result.distance, expected_distances[i], 0.1);
  }
}

TEST_F(RetracingPathTest, TrajectoryFollowsRetracingPath)
{
  const double expected_distance = 0.5;

  for (const auto & robot_pose : robot_trajectory) {
    auto result = nav2_util::distanceFromPath(target_path, robot_pose);
    EXPECT_NEAR(result.distance, expected_distance, 1e-6);
  }
}

TEST_F(CloverleafPathTest, TrajectoryFollowsCloverleafLoop)
{
  for (const auto & robot_pose : robot_trajectory) {
    auto result = nav2_util::distanceFromPath(target_path, robot_pose);
    EXPECT_LT(result.distance, 0.25);
  }
}

TEST_F(RetracingCircleTest, TrajectoryFollowsRetracingCircle)
{
  const double expected_distance = 0.2;

  for (const auto & robot_pose : robot_trajectory) {
    auto result = nav2_util::distanceFromPath(target_path, robot_pose);
    EXPECT_NEAR(result.distance, expected_distance, 0.01);
  }
}

TEST_F(ZigZagPathTest, TrajectoryFollowsZigZagPath)
{
  for (const auto & robot_pose : robot_trajectory) {
    auto result = nav2_util::distanceFromPath(target_path, robot_pose);
    EXPECT_LT(result.distance, 1.0);
  }
}

TEST_F(HairpinTurnTest, TrajectoryFollowsHairpinTurn)
{
  for (const auto & robot_pose : robot_trajectory) {
    auto result = nav2_util::distanceFromPath(target_path, robot_pose);
    EXPECT_LT(result.distance, 1.5);
  }
}

class CuttingCornerWindowedTest : public CuttingCornerTest {};

TEST_F(CuttingCornerWindowedTest, WindowedSearch)
{
  size_t start_index = 0;
  const double search_window = 11.0;

  for (size_t i = 0; i < robot_trajectory.size(); ++i) {
    const auto & robot_pose = robot_trajectory[i];
    auto result = nav2_util::distanceFromPath(target_path, robot_pose, start_index, search_window);
    start_index = result.closest_segment_index;
    EXPECT_NEAR(result.distance, expected_distances[i], 0.15);
  }
}

class RetracingPathWindowedTest : public RetracingPathTest {};

TEST_F(RetracingPathWindowedTest, WindowedSearch)
{
  const double expected_distance = 0.5;
  const double search_window = 21.0;
  size_t start_index = 0;

  for (size_t i = 0; i < robot_trajectory.size(); ++i) {
    const auto & robot_pose = robot_trajectory[i];
    auto result = nav2_util::distanceFromPath(target_path, robot_pose, start_index, search_window);
    start_index = result.closest_segment_index;
    EXPECT_NEAR(result.distance, expected_distance, 1e-6);
  }
}

class ZigZagPathWindowedTest : public ZigZagPathTest {};

TEST_F(ZigZagPathWindowedTest, WindowedSearch)
{
  const double search_window = 12.0;
  size_t start_index = 0;

  for (size_t i = 0; i < robot_trajectory.size(); ++i) {
    const auto & robot_pose = robot_trajectory[i];
    auto result = nav2_util::distanceFromPath(target_path, robot_pose, start_index, search_window);
    start_index = result.closest_segment_index;
    EXPECT_LT(result.distance, 1.0);
  }
}

class HairpinTurnWindowedTest : public HairpinTurnTest {};

TEST_F(HairpinTurnWindowedTest, WindowedSearch)
{
  const double search_window = 13.0;
  size_t start_index = 0;

  for (size_t i = 0; i < robot_trajectory.size(); ++i) {
    const auto & robot_pose = robot_trajectory[i];
    auto result = nav2_util::distanceFromPath(target_path, robot_pose, start_index, search_window);
    start_index = result.closest_segment_index;
    EXPECT_LT(result.distance, 1.5);
  }
}

TEST(PathUtilsWindowedTest, EdgeCases)
{
  auto robot_pose = createPose(5.0, 5.0);
  nav_msgs::msg::Path test_path;
  test_path.poses.push_back(createPose(0.0, 0.0));
  test_path.poses.push_back(createPose(10.0, 0.0));

  auto result = nav2_util::distanceFromPath(test_path, robot_pose, 1, 5.0);
  EXPECT_NEAR(result.distance, 7.071, 0.01);

  result = nav2_util::distanceFromPath(test_path, robot_pose, 0, 0.0);
  EXPECT_NEAR(result.distance, 7.071, 0.01);

  result = nav2_util::distanceFromPath(test_path, robot_pose, 10, 5.0);
  EXPECT_EQ(result.distance, std::numeric_limits<double>::infinity());
}
