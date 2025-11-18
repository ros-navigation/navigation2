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
#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/simple_path_handler.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using nav2_controller::PathIterator;
using PathSegment = std::pair<PathIterator, PathIterator>;

class PathHandlerWrapper : public nav2_controller::SimplePathHandler
{
public:
  PathHandlerWrapper()
  : SimplePathHandler() {}

  void pruneGlobalPlanWrapper(nav_msgs::msg::Path & path, const PathIterator end)
  {
    return prunePlan(path, end);
  }

  double getCostmapMaxExtentWrapper()
  {
    return getCostmapMaxExtent();
  }

  PathSegment
  findPlanSegmentIteratorsWrapper(const geometry_msgs::msg::PoseStamped & pose)
  {
    return findPlanSegmentIterators(pose);
  }

  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrameWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return transformToGlobalPlanFrame(pose);
  }
  nav_msgs::msg::Path transformLocalPlanWrapper(
    const geometry_msgs::msg::PoseStamped & pose,
    const PathIterator & closest,
    const PathIterator & end)
  {
    return transformLocalPlan(pose, closest, end);
  }

  void setGlobalPlanUpToInversion(const nav_msgs::msg::Path & path)
  {
    global_plan_up_to_inversion_ = path;
  }

  bool isWithinInversionTolerancesWrapper(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return isWithinInversionTolerances(robot_pose);
  }

  nav_msgs::msg::Path & getInvertedPath()
  {
    return global_plan_up_to_inversion_;
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

  PathIterator it = rtn_path.poses.begin() + 5;
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
  auto results = costmap_ros->set_parameters_atomically(
    {rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("robot_base_frame", "base_link")});
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  // Test initialization and getting costmap basic metadata
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());
  EXPECT_EQ(handler.getCostmapMaxExtentWrapper(), 2.475);

  // Set tf between map odom and base_link
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);

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
  auto [closest, pruned_plan_end] = handler.findPlanSegmentIteratorsWrapper(robot_pose);
  auto transformed_plan = handler.transformLocalPlanWrapper(robot_pose, closest, pruned_plan_end);
  auto & path_inverted = handler.getInvertedPath();
  EXPECT_EQ(closest - path_inverted.poses.begin(), 25);
  handler.pruneGlobalPlanWrapper(path_inverted, closest);
  auto & path_pruned = handler.getInvertedPath();
  EXPECT_EQ(path_pruned.poses.size(), 75u);
}

TEST(PathHandlerTests, TestTransforms)
{
  PathHandlerWrapper handler;
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("dummy.max_robot_pose_search_dist", rclcpp::ParameterValue(99999.9));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  // Test basic transformations and path handling
  handler.initialize(node, node->get_logger(), "dummy", costmap_ros, costmap_ros->getTfBuffer());

  // Set tf between map odom and base_link
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  tf_broadcaster_->sendTransform(t);

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

  auto [closest_point, pruned_plan_end] = handler.findPlanSegmentIteratorsWrapper(robot_pose);
  auto path_out = handler.transformLocalPlanWrapper(robot_pose, closest_point, pruned_plan_end);

  // Put it all together
  auto final_path = handler.transformGlobalPlan(robot_pose);
  EXPECT_EQ(final_path.poses.size(), path_out.poses.size());
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
    rclcpp::Parameter("dummy.enforce_path_inversion", true),
    });

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("dummy.max_robot_pose_search_dist").as_double(), 100.0);
  EXPECT_EQ(node->get_parameter("dummy.inversion_xy_tolerance").as_double(), 200.0);
  EXPECT_EQ(node->get_parameter("dummy.inversion_yaw_tolerance").as_double(), 300.0);
  EXPECT_EQ(node->get_parameter("dummy.prune_distance").as_double(), 400.0);
  EXPECT_EQ(node->get_parameter("dummy.enforce_path_inversion").as_bool(), true);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
