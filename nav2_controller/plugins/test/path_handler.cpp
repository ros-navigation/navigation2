// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/feasible_path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

using namespace std::chrono_literals;

class PathHandlerWrapper : public nav2_controller::FeasiblePathHandler
{
public:
  PathHandlerWrapper()
  : FeasiblePathHandler() {}

  void pruneGlobalPlanWrapper(nav_msgs::msg::Path & path, const nav2_core::PathIterator end)
  {
    return prunePlan(path, end);
  }

  double getCostmapMaxExtentWrapper()
  {
    return getCostmapMaxExtent();
  }

  nav2_core::PathSegment
  findPlanSegmentWrapper(const geometry_msgs::msg::PoseStamped & pose)
  {
    return findPlanSegment(pose);
  }

  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrameWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return transformToGlobalPlanFrame(pose);
  }
  nav_msgs::msg::Path transformLocalPlanWrapper(
    const nav2_core::PathIterator & closest,
    const nav2_core::PathIterator & end)
  {
    return transformLocalPlan(closest, end);
  }

  void setGlobalPlanUpToInversion(const nav_msgs::msg::Path & path)
  {
    global_plan_up_to_constraint_ = path;
  }

  bool isWithinInversionTolerancesWrapper(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return isWithinInversionTolerances(robot_pose);
  }

  nav_msgs::msg::Path & getInvertedPath()
  {
    return global_plan_up_to_constraint_;
  }

  bool retainedStateNeedsValidation()
  {
    return retained_state_needs_validation_;
  }

  geometry_msgs::msg::PoseStamped getTransformedGoalWrapper(
    const builtin_interfaces::msg::Time & stamp)
  {
    return getTransformedGoal(stamp);
  }
};

TEST(PathHandlerTests, GetAndPrunePath)
{
  nav_msgs::msg::Path path;
  PathHandlerWrapper handler;

  path.header.frame_id = "fkframe";
  path.poses.resize(11);
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());
  handler.setPlan(path);
  auto rtn_path = handler.getPlan();
  EXPECT_EQ(path.header.frame_id, rtn_path.header.frame_id);
  EXPECT_EQ(path.poses.size(), rtn_path.poses.size());

  nav2_core::PathIterator it = rtn_path.poses.begin() + 5;
  handler.pruneGlobalPlanWrapper(rtn_path, it);
  EXPECT_EQ(rtn_path.poses.size(), 6u);
}

TEST(PathHandlerTests, TestBounds)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);
  auto results = costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});

  // Test initialization and getting costmap basic metadata
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());
  EXPECT_EQ(handler.getCostmapMaxExtentWrapper(), 2.5);

  // Set tf between map odom and base_link
  auto tf_broadcaster_ = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  // Test getting the global plans within a bounds window
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 25.0;

  handler.setPlan(path);
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);
  EXPECT_THROW(handler.transformLocalPlanWrapper(closest, pruned_plan_end), std::runtime_error);
  auto & path_inverted = handler.getInvertedPath();
  EXPECT_EQ(closest - path_inverted.poses.begin(), 25);
  EXPECT_EQ(path_inverted.poses.size(), 75u);
}

TEST(PathHandlerTests, SearchDistanceSmallerThanPoseSpacing)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(0.5));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);
  costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster_ = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(3);
  for (unsigned int i = 0; i != path.poses.size(); ++i) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 0.9;

  handler.setPlan(path);
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);
  auto & bounded_path = handler.getInvertedPath();
  EXPECT_EQ(closest, bounded_path.poses.begin());
  EXPECT_EQ(pruned_plan_end, bounded_path.poses.end());
}

TEST(PathHandlerTests, TestBoundsWithConstraintCheck)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  node->declare_parameter("dummy.enforce_path_inversion", true);
  node->declare_parameter("dummy.enforce_path_rotation", true);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);
  auto results = costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});

  // Test initialization and getting costmap basic metadata
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());
  EXPECT_EQ(handler.getCostmapMaxExtentWrapper(), 2.5);

  // Set tf between map odom and base_link
  auto tf_broadcaster_ = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  // Test getting the global plans within a bounds window
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (unsigned int i = 0; i != 50; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 50; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 50 - i;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 50.0;

  handler.setPlan(path);
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);
  EXPECT_THROW(handler.transformLocalPlanWrapper(closest, pruned_plan_end), std::runtime_error);
  auto & path_inverted = handler.getInvertedPath();
  EXPECT_EQ(closest - path_inverted.poses.begin(), 49);
  EXPECT_EQ(path_inverted.poses.size(), 49u);
}

TEST(PathHandlerTests, TestTransforms)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  node->declare_parameter("dummy.reject_unit_path", rclcpp::ParameterValue(true));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  // Test basic transformations and path handling
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  // Set tf between map odom and base_link
  auto tf_broadcaster_ = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 2.5;

  EXPECT_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose), std::runtime_error);
  handler.setPlan(path);
  EXPECT_NO_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose));
  path.poses.resize(1);
  handler.setPlan(path);
  EXPECT_THROW(handler.transformToGlobalPlanFrameWrapper(robot_pose), std::runtime_error);
}

TEST(PathHandlerTests, TestInversionToleranceChecks)
{
  nav_msgs::msg::Path path;
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    path.poses.push_back(pose);
  }
  path.poses.back().pose.orientation.w = 1;

  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());
  handler.setGlobalPlanUpToInversion(path);

  // Not near (0,0)
  geometry_msgs::msg::PoseStamped robot_pose;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // Exactly on top of it
  robot_pose.pose.position.x = 9;
  robot_pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // Laterally of it
  robot_pose.pose.position.y = 9;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // On top but off angled
  robot_pose.pose.position.y = 0;
  robot_pose.pose.orientation.z = 0.8509035;
  robot_pose.pose.orientation.w = 0.525322;
  EXPECT_FALSE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // On top but off angled within tolerances
  robot_pose.pose.position.y = 0;
  robot_pose.pose.orientation.w = 0.9961947;
  robot_pose.pose.orientation.z = 0.0871558;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));

  // Offset spatially + off angled but both within tolerances
  robot_pose.pose.position.x = 9.10;
  EXPECT_TRUE(handler.isWithinInversionTolerancesWrapper(robot_pose));
}

TEST(PathHandlerTests, TestTransformedGoal)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());
  builtin_interfaces::msg::Time stamp;
  nav_msgs::msg::Path path;
  path.poses.resize(11);
  handler.setPlan(path);
  EXPECT_THROW(handler.getTransformedGoal(stamp), std::runtime_error);
  for (unsigned int i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = i;
    path.poses[i].header.frame_id = "map";
  }
  EXPECT_THROW(handler.getTransformedGoal(stamp), std::runtime_error);
}

TEST(PathHandlerTests, TestDynamicParams)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically({
    rclcpp::Parameter("dummy.max_robot_pose_search_dist", 100.0),
    rclcpp::Parameter("dummy.inversion_xy_tolerance", 200.0),
    rclcpp::Parameter("dummy.inversion_yaw_tolerance", 300.0),
    rclcpp::Parameter("dummy.prune_distance", 400.0),
    rclcpp::Parameter("dummy.minimum_rotation_angle", 500.0),
    rclcpp::Parameter("dummy.enforce_path_inversion", true),
    rclcpp::Parameter("dummy.enforce_path_rotation", true),
    });

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("dummy.max_robot_pose_search_dist").as_double(), 100.0);
  EXPECT_EQ(node->get_parameter("dummy.inversion_xy_tolerance").as_double(), 200.0);
  EXPECT_EQ(node->get_parameter("dummy.inversion_yaw_tolerance").as_double(), 300.0);
  EXPECT_EQ(node->get_parameter("dummy.prune_distance").as_double(), 400.0);
  EXPECT_EQ(node->get_parameter("dummy.minimum_rotation_angle").as_double(), 500.0);
  EXPECT_EQ(node->get_parameter("dummy.enforce_path_inversion").as_bool(), true);
  EXPECT_EQ(node->get_parameter("dummy.enforce_path_rotation").as_bool(), true);

  // Test setting invalid values
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("dummy.max_robot_pose_search_dist", -1.0)}
  );
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);
  // Value should remain unchanged
  EXPECT_EQ(node->get_parameter("dummy.max_robot_pose_search_dist").as_double(), 100.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("dummy.inversion_xy_tolerance", -1.0)}
  );
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);
  // Value should remain unchanged
  EXPECT_EQ(node->get_parameter("dummy.inversion_xy_tolerance").as_double(), 200.0);
}

// Builds a costmap whose bounds are stated by the test, so that the poses used for retained
// state validation are unambiguously inside or outside of it.
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> createBoundedCostmap(double origin, int size_meters)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link"),
      rclcpp::Parameter("width", size_meters),
      rclcpp::Parameter("height", size_meters),
      rclcpp::Parameter("resolution", 0.1),
      rclcpp::Parameter("origin_x", origin),
      rclcpp::Parameter("origin_y", origin)});

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);
  return costmap_ros;
}

nav_msgs::msg::Path createStraightPath(unsigned int num_poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(num_poses);
  for (unsigned int i = 0; i != num_poses; ++i) {
    path.poses[i].header.frame_id = "map";
    path.poses[i].pose.position.x = i;
    path.poses[i].pose.orientation.w = 1.0;
  }
  return path;
}

// Tracks the path in one metre steps, pruning the working plan on every step
void followPathTo(PathHandlerWrapper & handler, double x_end)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  for (double x = 0.0; x <= x_end; x += 1.0) {
    robot_pose.pose.position.x = x;
    auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);
    handler.transformLocalPlanWrapper(closest, pruned_plan_end);
  }
}

TEST(PathHandlerTests, RetainPruneStateOnIdenticalPlanWithinCostmap)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-10.0, 20);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  auto path = createStraightPath(31);
  handler.setPlan(path);
  followPathTo(handler, 5.0);

  // The robot is now further along the path than a bounded search from the start could reach
  ASSERT_EQ(handler.getInvertedPath().poses.size(), 26u);
  ASSERT_EQ(handler.getInvertedPath().poses.front().pose.position.x, 5.0);

  handler.setPlan(path);
  EXPECT_TRUE(handler.retainedStateNeedsValidation());

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 5.0;
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);

  EXPECT_FALSE(handler.retainedStateNeedsValidation());
  EXPECT_EQ(handler.getInvertedPath().poses.size(), 26u);
  EXPECT_EQ(closest, handler.getInvertedPath().poses.begin());
  EXPECT_EQ(closest->pose.position.x, 5.0);
  EXPECT_NE(pruned_plan_end, closest);
}

TEST(PathHandlerTests, ResetRetainedStateWhenCandidateLeavesCostmap)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-5.0, 10);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  auto path = createStraightPath(31);
  handler.setPlan(path);

  // Stands in for having tracked the path beyond the costmap before the goal was reissued
  nav_msgs::msg::Path retained_plan = path;
  retained_plan.poses.erase(retained_plan.poses.begin(), retained_plan.poses.begin() + 8);
  handler.setGlobalPlanUpToInversion(retained_plan);

  handler.setPlan(path);
  EXPECT_TRUE(handler.retainedStateNeedsValidation());

  // Teleoperated back to the start, the retained poses are no longer within the costmap
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 0.0;
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);

  EXPECT_FALSE(handler.retainedStateNeedsValidation());
  EXPECT_EQ(handler.getInvertedPath().poses.size(), path.poses.size());
  EXPECT_EQ(closest, handler.getInvertedPath().poses.begin());
  EXPECT_EQ(closest->pose.position.x, 0.0);
  EXPECT_NE(pruned_plan_end, closest);
}

TEST(PathHandlerTests, RetainStateAfterLocalRecoveryMotion)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-10.0, 20);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  auto path = createStraightPath(31);
  handler.setPlan(path);
  followPathTo(handler, 5.0);
  handler.setPlan(path);

  // Backed up during a recovery, but still alongside the retained portion of the path
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 4.6;
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);

  EXPECT_EQ(handler.getInvertedPath().poses.size(), 26u);
  EXPECT_EQ(closest->pose.position.x, 5.0);
  EXPECT_NE(pruned_plan_end, closest);
}

TEST(PathHandlerTests, ReplaceRetainedStateWhenPlanDiffers)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-10.0, 20);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  auto path = createStraightPath(31);
  handler.setPlan(path);
  followPathTo(handler, 5.0);
  ASSERT_EQ(handler.getInvertedPath().poses.size(), 26u);

  auto different_path = createStraightPath(31);
  different_path.poses[10].pose.position.y = 1.0;
  handler.setPlan(different_path);

  EXPECT_FALSE(handler.retainedStateNeedsValidation());
  EXPECT_EQ(handler.getInvertedPath().poses.size(), different_path.poses.size());
  EXPECT_EQ(handler.getInvertedPath().poses.front().pose.position.x, 0.0);
}

TEST(PathHandlerTests, ResetClearsRetainedState)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-10.0, 20);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  auto path = createStraightPath(31);
  handler.setPlan(path);
  followPathTo(handler, 5.0);
  ASSERT_EQ(handler.getInvertedPath().poses.size(), 26u);

  handler.reset();
  EXPECT_TRUE(handler.getInvertedPath().poses.empty());

  // The same geometry after a reset is a new navigation
  handler.setPlan(path);
  EXPECT_FALSE(handler.retainedStateNeedsValidation());
  EXPECT_EQ(handler.getInvertedPath().poses.size(), path.poses.size());
  EXPECT_EQ(handler.getInvertedPath().poses.front().pose.position.x, 0.0);
}

TEST(PathHandlerTests, RetainedStateValidationStaysPendingOnTfFailure)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-10.0, 20);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  // The plan frame is available, the costmap frame is not
  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  auto path = createStraightPath(31);
  handler.setPlan(path);
  handler.setPlan(path);
  ASSERT_TRUE(handler.retainedStateNeedsValidation());

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  EXPECT_THROW(
    handler.findPlanSegmentWrapper(robot_pose), nav2_core::ControllerTFError);
  EXPECT_TRUE(handler.retainedStateNeedsValidation());
  EXPECT_EQ(handler.getInvertedPath().poses.size(), path.poses.size());

  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  EXPECT_NO_THROW(handler.findPlanSegmentWrapper(robot_pose));
  EXPECT_FALSE(handler.retainedStateNeedsValidation());
}

TEST(PathHandlerTests, RetainedStateValidationKeepsOrderedSearch)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  auto costmap_ros = createBoundedCostmap(-10.0, 40);
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  auto tf_broadcaster = nav2::create_transform_broadcaster(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster->sendTransform(t);
  t.child_frame_id = "odom";
  tf_broadcaster->sendTransform(t);
  std::this_thread::sleep_for(10ms);

  // A path that comes back alongside where it started
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (unsigned int i = 0; i != 11; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 11; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 10 - i;
    pose.pose.position.y = 0.5;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  handler.setPlan(path);
  nav_msgs::msg::Path retained_plan = path;
  retained_plan.poses.erase(retained_plan.poses.begin(), retained_plan.poses.begin() + 18);
  handler.setGlobalPlanUpToInversion(retained_plan);
  handler.setPlan(path);

  // The outbound leg is closer to the robot, but has already been traversed
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 3.0;
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);

  EXPECT_EQ(handler.getInvertedPath().poses.size(), retained_plan.poses.size());
  EXPECT_EQ(closest->pose.position.x, 3.0);
  EXPECT_EQ(closest->pose.position.y, 0.5);
  EXPECT_NE(pruned_plan_end, closest);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
