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
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_controller/plugins/feasible_path_handler.hpp"
#include "nav2_ros_common/tf_manager.hpp"

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
    global_plan_up_to_constraint_ = {path};
  }

  bool isWithinInversionTolerancesWrapper(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return isWithinInversionTolerances(robot_pose);
  }

  nav2_controller::PrunablePath & getInvertedPath()
  {
    return global_plan_up_to_constraint_;
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

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());
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
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());
  EXPECT_EQ(handler.getCostmapMaxExtentWrapper(), 2.5);

  // Set tf between map odom and base_link
  auto tf_manager_ = nav2::create_transform_manager(node, false);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_manager_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_manager_->sendTransform(t);
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
  EXPECT_EQ(closest - path_inverted.path.poses.begin(), 25);
  EXPECT_EQ(closest, path_inverted.begin());
  EXPECT_EQ(path_inverted.size(), 75u);
  EXPECT_EQ(path_inverted.path.poses.size(), 100u);
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
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());

  auto tf_manager_ = nav2::create_transform_manager(node, false);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_manager_->sendTransform(t);
  t.child_frame_id = "odom";
  tf_manager_->sendTransform(t);
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
  EXPECT_EQ(closest, bounded_path.begin());
  EXPECT_EQ(pruned_plan_end, bounded_path.end());
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
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());
  EXPECT_EQ(handler.getCostmapMaxExtentWrapper(), 2.5);

  // Set tf between map odom and base_link
  auto tf_manager_ = nav2::create_transform_manager(node, false);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_manager_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_manager_->sendTransform(t);
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
  auto & path_inverted = handler.getInvertedPath();
  EXPECT_EQ(closest - path_inverted.path.poses.begin(), 49);
  EXPECT_THROW(handler.transformLocalPlanWrapper(closest, pruned_plan_end), std::runtime_error);
  EXPECT_EQ(path_inverted.start_index, 0u);
  EXPECT_EQ(path_inverted.size(), 49u);
}

TEST(PathHandlerTests, TestPruningKeepsPosesUntilNewPlan)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);
  costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.rotation.w = 1.0;
  ASSERT_TRUE(costmap_ros->getTfManager()->getRawBuffer()->setTransform(t, "test", true));

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(100);
  for (unsigned int i = 0; i != path.poses.size(); ++i) {
    path.poses[i].pose.position.x = 0.1 * i;
    path.poses[i].header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";

  // Pruning only moves the start of the path, the passed poses are kept
  handler.setPlan(path);
  auto & pruned_path = handler.getInvertedPath();
  for (const unsigned int index : {10u, 20u}) {
    robot_pose.pose.position.x = 0.1 * index;
    auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);
    EXPECT_NO_THROW(handler.transformLocalPlanWrapper(closest, pruned_plan_end));
    EXPECT_EQ(pruned_path.begin(), closest);
    EXPECT_EQ(pruned_path.start_index, index);
    EXPECT_EQ(pruned_path.path.poses.size(), path.poses.size());
  }

  // The search does not go back into the pruned poses
  robot_pose.pose.position.x = 0.0;
  auto [closest, pruned_plan_end] = handler.findPlanSegmentWrapper(robot_pose);
  EXPECT_EQ(closest, pruned_path.begin());

  // A new plan starts unpruned
  handler.setPlan(path);
  EXPECT_EQ(pruned_path.begin(), pruned_path.path.poses.begin());
  EXPECT_EQ(pruned_path.size(), path.poses.size());
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
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());

  // Set tf between map odom and base_link
  auto tf_manager_ = nav2::create_transform_manager(node, false);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_manager_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_manager_->sendTransform(t);
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

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());
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

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());
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

  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfManager());

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

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
