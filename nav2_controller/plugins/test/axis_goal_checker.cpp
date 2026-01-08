// Copyright (c) 2025 Dexory
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
#include <string>
#include <cmath>

#include "gtest/gtest.h"
#include "nav2_controller/plugins/axis_goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"

using nav2_controller::AxisGoalChecker;

class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name)
  {
  }

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }
};

// Helper function to create a path with multiple poses
nav_msgs::msg::Path createPath(const std::vector<std::pair<double, double>> & positions)
{
  nav_msgs::msg::Path path;
  for (const auto & pos : positions) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = pos.first;
    pose.pose.position.y = pos.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(AxisGoalChecker, reset)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  agc.reset();
  EXPECT_TRUE(true);
}

TEST(AxisGoalChecker, initialize_and_tolerances)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;

  // Test tolerance API
  EXPECT_TRUE(agc.getTolerances(pose_tol, vel_tol));
  // Default tolerances should be 0.25
  EXPECT_EQ(pose_tol.position.x, 0.25);
  EXPECT_EQ(pose_tol.position.y, 0.25);
}

TEST(AxisGoalChecker, dynamic_parameters)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  // Test dynamic parameters
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.along_path_tolerance", 0.5),
      rclcpp::Parameter("test.cross_track_tolerance", 0.3),
      rclcpp::Parameter("test.path_length_tolerance", 2.0),
      rclcpp::Parameter("test.is_overshoot_valid", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.along_path_tolerance").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("test.cross_track_tolerance").as_double(), 0.3);
  EXPECT_EQ(node->get_parameter("test.path_length_tolerance").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.is_overshoot_valid").as_bool(), true);

  // Test that tolerances are updated
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  EXPECT_TRUE(agc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 0.3);  // min of along_path and cross_track
  EXPECT_EQ(pose_tol.position.y, 0.3);
}

TEST(AxisGoalChecker, single_point_path)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  // Create a path with only one point (goal)
  nav_msgs::msg::Path path = createPath({{0.0, 0.0}});

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path));

  // Robot within tolerance
  query_pose.position.x = 0.2;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path));

  // Robot outside combined tolerance (hypot(0.25, 0.25) ≈ 0.354)
  query_pose.position.x = 0.36;
  query_pose.position.y = 0.0;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, path));
}

TEST(AxisGoalChecker, straight_path_along_x)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a short path (within path_length_tolerance) for checking
  nav_msgs::msg::Path short_path = createPath({{1.5, 0.0}, {2.0, 0.0}});

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot slightly before goal (within along_path_tolerance)
  query_pose.position.x = 1.8;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot too far before goal (outside along_path_tolerance)
  query_pose.position.x = 1.74;
  query_pose.position.y = 0.0;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with small cross-track error (within cross_track_tolerance)
  query_pose.position.x = 2.0;
  query_pose.position.y = 0.2;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with large cross-track error (outside cross_track_tolerance)
  query_pose.position.x = 2.0;
  query_pose.position.y = 0.3;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot slightly past goal (should fail without is_overshoot_valid)
  query_pose.position.x = 2.26;
  query_pose.position.y = 0.0;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, straight_path_along_y)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.0;
  goal_pose.position.y = 2.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a short path (within path_length_tolerance) for checking
  nav_msgs::msg::Path short_path = createPath({{0.0, 1.5}, {0.0, 2.0}});

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot slightly before goal (within along_path_tolerance)
  query_pose.position.x = 0.0;
  query_pose.position.y = 1.8;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with cross-track error (within tolerance)
  query_pose.position.x = 0.2;
  query_pose.position.y = 2.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with large cross-track error (outside tolerance)
  query_pose.position.x = 0.3;
  query_pose.position.y = 2.0;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, diagonal_path)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 2.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a short diagonal path (within path_length_tolerance) for checking
  nav_msgs::msg::Path short_path = createPath({{1.65, 1.65}, {2.0, 2.0}});

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot slightly before goal along the diagonal (within along_path_tolerance)
  // Moving 0.2 along the path at 45 degrees: dx = dy = 0.2 * cos(45) ≈ 0.14
  double delta = 0.14;
  query_pose.position.x = 2.0 - delta;
  query_pose.position.y = 2.0 - delta;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with cross-track error perpendicular to path
  // For a 45-degree path, perpendicular offset at +45 degrees to the right
  query_pose.position.x = 2.0 + 0.1;
  query_pose.position.y = 2.0 - 0.1;  // perpendicular to the path
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with large cross-track error
  query_pose.position.x = 2.0 + 0.2;
  query_pose.position.y = 2.0 - 0.2;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, overshoot_valid)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  // Enable overshoot
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.is_overshoot_valid", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a short path along x-axis (within path_length_tolerance) for checking
  nav_msgs::msg::Path short_path = createPath({{1.5, 0.0}, {2.0, 0.0}});

  // Robot slightly past goal (should pass with is_overshoot_valid=true)
  query_pose.position.x = 2.15;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot far past goal (should still pass with overshoot valid - any overshoot allowed)
  query_pose.position.x = 10.0;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot before goal (should still pass)
  query_pose.position.x = 1.8;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, path_length_tolerance)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  // Create a long path that exceeds path_length_tolerance (default 1.0)
  nav_msgs::msg::Path long_path = createPath({
    {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}
  });

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  query_pose.position.x = 1.9;
  query_pose.position.y = 0.0;
  query_pose.position.z = 0.0;
  query_pose.orientation.w = 1.0;

  geometry_msgs::msg::Twist velocity;

  // Path length is 2.0, exceeds tolerance of 1.0, so should return false
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, long_path));

  // Create a short path within tolerance
  nav_msgs::msg::Path short_path = createPath({{1.5, 0.0}, {2.0, 0.0}});
  // Path length is 0.5, within tolerance of 1.0, so should check goal
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, combined_errors)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a short path along x-axis (within path_length_tolerance) for checking
  nav_msgs::msg::Path short_path = createPath({{1.5, 0.0}, {2.0, 0.0}});

  // Robot with both along-path and cross-track errors, both within tolerance
  query_pose.position.x = 1.9;  // 0.1 along-path error
  query_pose.position.y = 0.15;  // 0.15 cross-track error
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with along-path error within tolerance but cross-track error outside
  query_pose.position.x = 1.9;
  query_pose.position.y = 0.3;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with cross-track error within tolerance but along-path error outside
  query_pose.position.x = 1.7;
  query_pose.position.y = 0.15;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, angled_approach)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  // Create a path with a 30-degree angle
  double angle = M_PI / 6.0;  // 30 degrees

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0 * cos(angle);
  goal_pose.position.y = 2.0 * sin(angle);
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a short path (within path_length_tolerance) for checking
  nav_msgs::msg::Path short_path = createPath({
    {1.65 * cos(angle), 1.65 * sin(angle)},
    {2.0 * cos(angle), 2.0 * sin(angle)}
  });

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot slightly before goal along the path direction
  double delta = 0.15;
  query_pose.position.x = goal_pose.position.x - delta * cos(angle);
  query_pose.position.y = goal_pose.position.y - delta * sin(angle);
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot with cross-track offset (perpendicular to path)
  double cross_offset = 0.15;
  query_pose.position.x = goal_pose.position.x + cross_offset * cos(angle + M_PI / 2.0);
  query_pose.position.y = goal_pose.position.y + cross_offset * sin(angle + M_PI / 2.0);
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, identical_consecutive_poses)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  // Create a path where the last two poses are identical
  nav_msgs::msg::Path path_with_duplicates = createPath({{1.5, 0.0}, {2.0, 0.0}, {2.0, 0.0}});

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Robot within tolerance should succeed (falls back to simple distance check)
  query_pose.position.x = 1.85;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path_with_duplicates));

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path_with_duplicates));

  // Robot outside tolerance should fail
  query_pose.position.x = 1.7;
  query_pose.position.y = 0.0;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, path_with_duplicates));
}

TEST(AxisGoalChecker, robot_at_goal_position)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a normal path
  nav_msgs::msg::Path short_path = createPath({{1.5, 0.0}, {2.0, 0.0}});

  // Robot exactly at goal position
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));

  // Robot extremely close to goal (within numerical precision threshold)
  query_pose.position.x = 2.0 + 1e-7;
  query_pose.position.y = 0.0 + 1e-7;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, short_path));
}

TEST(AxisGoalChecker, multiple_consecutive_poses_too_close_to_goal)
{
  auto node = std::make_shared<TestLifecycleNode>("axis_goal_checker_test");
  AxisGoalChecker agc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  agc.initialize(node, "test", costmap);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2.0;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.0;
  goal_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose query_pose;
  geometry_msgs::msg::Twist velocity;

  // Create a path where multiple consecutive poses at the end are extremely close to goal
  // (within 1e-6 distance threshold), but there's a valid pose further back
  // Keep path short to stay within path_length_tolerance
  nav_msgs::msg::Path path_with_close_poses = createPath({
    {1.8, 0.0},                // Valid pose before goal
    {2.0 - 0.001, 0.0},        // Valid pose 1mm before goal (> 1e-6 threshold)
    {2.0 + 1e-7, 0.0},         // Too close to goal
    {2.0 + 5e-8, 1e-8},        // Too close to goal
    {2.0, 0.0}                 // Goal position (identical)
  });

  // Robot within tolerance - should use the valid pose at (2.0-0.001, 0.0)
  // to determine path direction
  query_pose.position.x = 1.85;
  query_pose.position.y = 0.0;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path_with_close_poses));

  // Robot at goal
  query_pose = goal_pose;
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path_with_close_poses));

  // Robot outside tolerance should fail
  query_pose.position.x = 1.7;
  query_pose.position.y = 0.0;
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, path_with_close_poses));

  // Create a path where ALL poses are too close to goal (should fall back to distance check)
  nav_msgs::msg::Path path_all_close = createPath({
    {2.0 + 1e-7, 0.0},
    {2.0 + 5e-8, 1e-8},
    {2.0, 0.0}
  });

  // Robot within combined tolerance should succeed (fallback to distance check)
  query_pose.position.x = 2.0 + 0.2;
  query_pose.position.y = 0.15;
  double combined_tolerance = std::hypot(0.25, 0.25);
  double distance = std::hypot(0.2, 0.15);
  EXPECT_LT(distance, combined_tolerance);  // Verify we're within tolerance
  EXPECT_TRUE(agc.isGoalReached(query_pose, goal_pose, velocity, path_all_close));

  // Robot outside combined tolerance should fail
  query_pose.position.x = 2.0 + 0.3;
  query_pose.position.y = 0.3;
  distance = std::hypot(0.3, 0.3);
  EXPECT_GT(distance, combined_tolerance);  // Verify we're outside tolerance
  EXPECT_FALSE(agc.isGoalReached(query_pose, goal_pose, velocity, path_all_close));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
