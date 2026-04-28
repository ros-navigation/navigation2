// Copyright (c) 2022 Joshua Wallace
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
// limitations under the License. Reserved.

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planner_tester.hpp"

using nav2_system_tests::PlannerTester;
using nav2_util::TestCostmap;

TEST(testIsPathValid, testIsPathValid)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::top_left_obstacle);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // empty path
  auto response = planner_tester->isPathValid(path, max_cost, consider_unknown_as_obstacle);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->is_valid);

  // invalid path
  for (float i = 0; i < 10; i += 1.0) {
    for (float j = 0; j < 10; j += 1.0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i;
      pose.pose.position.y = j;
      path.poses.push_back(pose);
    }
  }
  response = planner_tester->isPathValid(path, max_cost, consider_unknown_as_obstacle);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);

  // valid path
  path.poses.clear();
  for (float i = 0; i < 10; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = i;
    path.poses.push_back(pose);
  }
  response = planner_tester->isPathValid(path, max_cost, consider_unknown_as_obstacle);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_valid);

  // valid path, but contains NO_INFORMATION(255)
  path.poses.clear();
  consider_unknown_as_obstacle = true;
  for (float i = 0; i < 10; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = i;
    path.poses.push_back(pose);
  }
  response = planner_tester->isPathValid(path, max_cost, consider_unknown_as_obstacle);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);

  // valid path but higher than max cost
  max_cost = 0;
  response = planner_tester->isPathValid(path, max_cost, consider_unknown_as_obstacle);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);
}

TEST(testIsPathValid, testInvalidPoseIndices)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::top_left_obstacle);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create path that goes through obstacle
  for (float i = 0; i < 10; i += 1.0) {
    for (float j = 0; j < 10; j += 1.0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i;
      pose.pose.position.y = j;
      path.poses.push_back(pose);
    }
  }

  auto response = planner_tester->isPathValid(path, max_cost, consider_unknown_as_obstacle);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);
  EXPECT_FALSE(response->invalid_pose_indices.empty());
  // The first invalid pose should be at the obstacle
  EXPECT_GT(response->invalid_pose_indices[0], 0);
}

TEST(testIsPathValid, testCustomFootprint)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::open_space);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create a simple path
  for (float i = 1.0; i < 5.0; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = 5.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  // Test with custom footprint - square footprint
  std::string footprint = "[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]";
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", footprint);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_valid);
}

TEST(testIsPathValid, testInvalidFootprint)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::open_space);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create a simple path
  for (float i = 1.0; i < 5.0; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = 5.0;
    path.poses.push_back(pose);
  }

  // Test with invalid footprint string
  std::string invalid_footprint = "invalid_footprint_string";
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", invalid_footprint);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
}

TEST(testIsPathValid, testLayerName)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::open_space);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create a simple path
  for (float i = 1.0; i < 5.0; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = 5.0;
    path.poses.push_back(pose);
  }

  // Test with non-existent layer name
  std::string layer_name = "non_existent_layer";
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, layer_name);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->is_valid);
}

TEST(testIsPathValid, testEmptyLayerName)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::open_space);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create a simple path
  for (float i = 1.0; i < 5.0; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = 5.0;
    path.poses.push_back(pose);
  }

  // Test with empty layer name (should use full costmap)
  std::string layer_name = "";
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, layer_name);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_valid);
}

TEST(testIsPathValid, testFootprintCollisionChecking)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::top_left_obstacle);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create a path near the obstacle
  for (float i = 2.0; i < 5.0; i += 0.5) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = 2.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  // Test with larger footprint that should collide
  std::string large_footprint = "[[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]]";
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", large_footprint);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  // Path might be invalid due to large footprint near obstacle
  if (!response->is_valid) {
    EXPECT_FALSE(response->invalid_pose_indices.empty());
  }
}

TEST(testIsPathValid, testStopAtFirstCollision)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::top_left_obstacle);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create path with multiple points that go through obstacles
  for (float i = 0; i < 10; i += 1.0) {
    for (float j = 0; j < 10; j += 1.0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i;
      pose.pose.position.y = j;
      path.poses.push_back(pose);
    }
  }

  // Test with stop_at_first_collision = true (default, stops at first invalid pose)
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", "", true);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);
  EXPECT_EQ(response->invalid_pose_indices.size(), 1u);

  // Test with stop_at_first_collision = false (checks all poses)
  response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", "", false);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);
  EXPECT_GT(response->invalid_pose_indices.size(), 1u);
}

TEST(testIsPathValid, testMaxLookaheadDistance)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::top_left_obstacle);

  nav_msgs::msg::Path path;
  unsigned int max_cost = 253;
  bool consider_unknown_as_obstacle = false;

  // Create a long straight path along y-axis at x=1.0 (clear of top-left obstacle)
  for (float i = 0; i < 10; i += 0.5) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = i;
    path.poses.push_back(pose);
  }

  // Full path should be valid (no obstacle at x=1.0)
  auto response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", "", true, -1.0);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_valid);

  // With a short lookahead distance, should also be valid (subset of valid path)
  response = planner_tester->isPathValid(
    path, max_cost, consider_unknown_as_obstacle, "", "", true, 2.0);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_valid);

  // Now create a path that goes through the obstacle area
  nav_msgs::msg::Path obstacle_path;
  for (float i = 0; i < 10; i += 1.0) {
    for (float j = 0; j < 10; j += 1.0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i;
      pose.pose.position.y = j;
      obstacle_path.poses.push_back(pose);
    }
  }

  // Full path validation should find obstacles
  response = planner_tester->isPathValid(
    obstacle_path, max_cost, consider_unknown_as_obstacle, "", "", true, -1.0);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_valid);

  // With a very short lookahead, may not reach the obstacle
  response = planner_tester->isPathValid(
    obstacle_path, max_cost, consider_unknown_as_obstacle, "", "", true, 0.5);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  // With short lookahead, the path segment validated may not reach obstacles
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  return all_successful;
}
