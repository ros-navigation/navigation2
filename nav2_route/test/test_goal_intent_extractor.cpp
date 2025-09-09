// Copyright (c) 2025, Open Navigation LLC
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_route/goal_intent_extractor.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT


class GoalIntentExtractorWrapper : public GoalIntentExtractor
{
public:
  GoalIntentExtractorWrapper() = default;

  // API to set the start/goal poses that would be set normally
  // via `findStartandGoal` when called sequentially, so we can test
  // `pruneStartandGoal` independent for many cases easily
  void setStartAndGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    start_ = start;
    goal_ = goal;
  }

  geometry_msgs::msg::PoseStamped getStart()
  {
    return start_;
  }
};

TEST(GoalIntentExtractorTest, test_obj_lifecycle)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("goal_intent_extractor_test");
  GoalIntentExtractor extractor;
  Graph graph;
  GraphToIDMap id_map;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber = nullptr;
  extractor.configure(node, graph, &id_map, nullptr, costmap_subscriber, "map", "map", "base_link");
}

TEST(GoalIntentExtractorTest, test_transform_pose)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("goal_intent_extractor_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  GoalIntentExtractor extractor;
  Graph graph;
  GraphToIDMap id_map;
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf->setCreateTimerInterface(timer_interface);
  auto transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf);
  tf2_ros::TransformBroadcaster broadcaster(node);
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber = nullptr;
  extractor.configure(node, graph, &id_map, tf, costmap_subscriber, "map", "map", "base_link");

  // Test transformations same frame, should pass
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  EXPECT_NO_THROW(extractor.transformPose(pose, "map"));

  // Test transformations when nothing on TF buffer of different frames
  pose.header.frame_id = "gps";
  EXPECT_THROW(extractor.transformPose(pose, "map"), nav2_core::RouteTFError);

  // Now transforms are available, should work
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = node->now();
  transform.child_frame_id = "gps";
  broadcaster.sendTransform(transform);
  EXPECT_NO_THROW(extractor.transformPose(pose, "map"));
}

TEST(GoalIntentExtractorTest, test_start_goal_finder)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("goal_intent_extractor_test");
  node->declare_parameter("enable_nn_search", rclcpp::ParameterValue(false));
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  GoalIntentExtractorWrapper extractor;
  Graph graph;
  GraphToIDMap id_map;
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf->setCreateTimerInterface(timer_interface);

  // Make a 3x3 graph of points 0,0 -> 2,2 (ROS logo)
  graph.resize(9);
  unsigned int idx = 0;
  unsigned int ids = 1;
  for (unsigned int i = 0; i != 3; i++) {
    for (unsigned int j = 0; j != 3; j++) {
      graph[idx].nodeid = ids;
      graph[idx].coords.x = i;
      graph[idx].coords.y = j;
      id_map[graph[idx].nodeid] = idx;
      idx++;
      ids++;
    }
  }
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber = nullptr;
  extractor.configure(node, graph, &id_map, tf, costmap_subscriber, "map", "map", "base_link");

  // Test sending goal and start IDs to search
  nav2_msgs::action::ComputeRoute::Goal raw_goal;
  raw_goal.start_id = 1;
  raw_goal.goal_id = 9;
  raw_goal.use_poses = false;
  auto goal = std::make_shared<const nav2_msgs::action::ComputeRoute::Goal>(raw_goal);
  auto [start1, goal1] = extractor.findStartandGoal(goal);
  EXPECT_EQ(start1, 0u);
  EXPECT_EQ(goal1, 8u);

  // Set and check reset
  geometry_msgs::msg::PoseStamped p1;
  p1.pose.position.x = 45.0;
  auto start = extractor.getStart();
  EXPECT_EQ(start.pose.position.x, 0.0);
  EXPECT_EQ(start.pose.position.y, 0.0);
  extractor.overrideStart(p1);
  start = extractor.getStart();
  EXPECT_EQ(start.pose.position.x, 45.0);

  // Test sending a goal with start/goal poses to find closest nodes to
  raw_goal.start.header.frame_id = "map";
  raw_goal.start.pose.position.x = -1.0;
  raw_goal.start.pose.position.y = -1.0;  // Closest on graph should be (0,0) -> 0
  raw_goal.goal.header.frame_id = "map";
  raw_goal.goal.pose.position.x = 1.0;
  raw_goal.goal.pose.position.y = 1.0;  // Closest on graph should be (1,1) -> 5
  raw_goal.use_start = true;
  raw_goal.use_poses = true;
  goal = std::make_shared<const nav2_msgs::action::ComputeRoute::Goal>(raw_goal);
  auto [start2, goal2] = extractor.findStartandGoal(goal);
  EXPECT_EQ(start2, 0u);
  EXPECT_EQ(goal2, 4u);

  // Test sending a goal without using start (TF lookup failure)
  raw_goal.use_start = false;
  raw_goal.use_poses = true;
  goal = std::make_shared<const nav2_msgs::action::ComputeRoute::Goal>(raw_goal);
  EXPECT_THROW(extractor.findStartandGoal(goal), nav2_core::RouteTFError);

  // Try with an empty graph, should fail to find nodes
  Graph empty_graph;
  GraphToIDMap empty_id_map;
  extractor.setGraph(empty_graph, &empty_id_map);
  raw_goal.use_start = true;
  raw_goal.use_poses = true;
  goal = std::make_shared<const nav2_msgs::action::ComputeRoute::Goal>(raw_goal);
  EXPECT_THROW(extractor.findStartandGoal(goal), nav2_core::IndeterminantNodesOnGraph);
}

TEST(GoalIntentExtractorTest, test_pruning)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("goal_intent_extractor_test");
  GoalIntentExtractorWrapper extractor;
  Graph graph;
  GraphToIDMap id_map;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber = nullptr;
  extractor.configure(node, graph, &id_map, nullptr, costmap_subscriber, "map", "map", "base_link");

  // Setup goal to use (only uses the use_poses field)
  nav2_msgs::action::ComputeRoute::Goal raw_goal;
  raw_goal.use_poses = false;
  auto no_poses_goal = std::make_shared<const nav2_msgs::action::ComputeRoute::Goal>(raw_goal);
  raw_goal.use_poses = true;
  auto poses_goal = std::make_shared<const nav2_msgs::action::ComputeRoute::Goal>(raw_goal);

  // Test that we return identical things if not using poses or route is empty of edges
  Route routeA;
  routeA.edges.resize(10);
  ReroutingState rerouting_info;
  rerouting_info.first_time = true;
  EXPECT_EQ(extractor.pruneStartandGoal(routeA, no_poses_goal, rerouting_info).edges.size(), 10u);
  routeA.edges.clear();
  EXPECT_EQ(extractor.pruneStartandGoal(routeA, poses_goal, rerouting_info).edges.size(), 0u);
  EXPECT_EQ(extractor.pruneStartandGoal(routeA, no_poses_goal, rerouting_info).edges.size(), 0u);

  // Create a sample route to test pruning upon with different start/goal pose requests
  Node node1, node2, node3, node4;
  node1.nodeid = 1;
  node1.coords.x = 0.0;
  node1.coords.y = 0.0;
  node1.search_state.traversal_cost = 0.0;
  node2.nodeid = 2;
  node2.coords.x = 1.0;
  node2.coords.y = 0.0;
  node2.search_state.traversal_cost = 1.0;
  node3.nodeid = 3;
  node3.coords.x = 2.0;
  node3.coords.y = 0.0;
  node3.search_state.traversal_cost = 1.0;
  node4.nodeid = 4;
  node4.coords.x = 3.0;
  node4.coords.y = 0.0;
  node4.search_state.traversal_cost = 1.0;

  DirectionalEdge edge1, edge2, edge3;
  edge1.edgeid = 5;
  edge1.start = &node1;
  edge1.end = &node2;
  edge2.edgeid = 6;
  edge2.start = &node2;
  edge2.end = &node3;
  edge3.edgeid = 7;
  edge3.start = &node3;
  edge3.end = &node4;

  Route route;
  route.route_cost = 3.0;
  route.start_node = &node1;
  route.edges.push_back(&edge1);
  route.edges.push_back(&edge2);
  route.edges.push_back(&edge3);

  // Test overlapping goal/start poses to their respective terminal points on route
  geometry_msgs::msg::PoseStamped start, goal;
  goal.pose.position.x = 3.0;
  extractor.setStartAndGoal(start, goal);
  EXPECT_EQ(extractor.pruneStartandGoal(route, poses_goal, rerouting_info).edges.size(), 3u);

  // Test orthogonally misaligned, so should still have all the same points
  start.pose.position.y = -10.0;
  goal.pose.position.y = 10.0;
  extractor.setStartAndGoal(start, goal);
  EXPECT_EQ(extractor.pruneStartandGoal(route, poses_goal, rerouting_info).edges.size(), 3u);

  // Test start node before start pose and end node before end pose (no pruning)
  start.pose.position.x = -1.0;
  goal.pose.position.x = 4.0;
  start.pose.position.y = 0.0;
  goal.pose.position.y = 0.0;
  extractor.setStartAndGoal(start, goal);
  auto rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 3u);

  // Test start, and only start is after the start node along edge1, should be pruned
  start.pose.position.x = 0.2;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 2u);
  EXPECT_EQ(rtn.edges[0]->edgeid, 6u);
  EXPECT_EQ(rtn.start_node->nodeid, 2u);
  EXPECT_EQ(rtn.route_cost, 2.0);

  // Test start, and only start is after the start node along edge1
  // should not be pruned, within min distance
  start.pose.position.x = 0.09;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 3u);
  EXPECT_EQ(rtn.edges[0]->edgeid, 5u);
  EXPECT_EQ(rtn.start_node->nodeid, 1u);
  EXPECT_EQ(rtn.route_cost, 3.0);

  // Test but now with no_poses_goal to test if we trigger with !first_time condition
  // when we should be able to prune due to a secondary trigger as the result of rerouting
  start.pose.position.x = 0.2;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, no_poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 2u);
  EXPECT_EQ(rtn.edges[0]->edgeid, 6u);
  EXPECT_EQ(rtn.start_node->nodeid, 2u);
  EXPECT_EQ(rtn.route_cost, 2.0);
  // But now if we have first_time, doesn't prune!
  rerouting_info.first_time = true;
  rtn = extractor.pruneStartandGoal(route, no_poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 3u);

  // Test end, and only end is before the end node along edge3, should be pruned
  start.pose.position.x = 0.0;
  goal.pose.position.x = 2.5;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 2u);
  EXPECT_EQ(rtn.edges.back()->edgeid, 6u);
  EXPECT_EQ(rtn.edges.back()->end->nodeid, 3u);
  EXPECT_EQ(rtn.route_cost, 2.0);

  // Test end, and only end is before the end node along edge3, should not be pruned
  // As within the min distance
  start.pose.position.x = 0.0;
  goal.pose.position.x = 2.9;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 3u);
  EXPECT_EQ(rtn.edges.back()->edgeid, 7u);
  EXPECT_EQ(rtn.edges.back()->end->nodeid, 4u);
  EXPECT_EQ(rtn.route_cost, 3.0);

  // Test both together can be pruned with realistic tracking offsets from route
  // Also, Check route info for resetting to nullptrs since not the same last edge
  start.pose.position.x = 0.6;
  start.pose.position.y = 0.4;
  goal.pose.position.x = 2.6;
  goal.pose.position.y = -0.4;
  rerouting_info.curr_edge = &edge3;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 1u);
  EXPECT_EQ(rtn.edges[0]->edgeid, 6u);
  EXPECT_EQ(rtn.edges.back()->start->nodeid, 2u);
  EXPECT_EQ(rtn.edges.back()->end->nodeid, 3u);
  EXPECT_EQ(rtn.route_cost, 1.0);
  EXPECT_EQ(rerouting_info.curr_edge, nullptr);

  // Test pruning with route information with same situation as above
  // But now check route info is retained because it IS the same edge as last time
  // & stores closest point to use
  rerouting_info.curr_edge = &edge1;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 1u);
  EXPECT_EQ(rtn.edges[0]->edgeid, 6u);
  EXPECT_EQ(rtn.edges.back()->start->nodeid, 2u);
  EXPECT_EQ(rtn.edges.back()->end->nodeid, 3u);
  EXPECT_EQ(rtn.route_cost, 1.0);
  EXPECT_EQ(rerouting_info.curr_edge->edgeid, 5u);
  EXPECT_NEAR(rerouting_info.closest_pt_on_edge.x, 0.6, 0.01);
  EXPECT_NEAR(rerouting_info.closest_pt_on_edge.y, 0.0, 0.01);

  // Test both together can be pruned but won't be due to huge offsets > max_dist_from_edge (8m)
  start.pose.position.x = 0.6;
  start.pose.position.y = 8.1;
  goal.pose.position.x = 2.6;
  goal.pose.position.y = -8.1;
  extractor.setStartAndGoal(start, goal);
  rtn = extractor.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges[0]->start->nodeid, 1u);
  EXPECT_EQ(rtn.edges.back()->end->nodeid, 4u);
  EXPECT_EQ(rtn.edges.size(), 3u);

  // Test the both pruned condition but goal is not pruned because prune_goal = false
  node->set_parameter(rclcpp::Parameter("prune_goal", rclcpp::ParameterValue(false)));
  start.pose.position.x = 0.6;
  start.pose.position.y = 0.4;
  goal.pose.position.x = 2.6;
  goal.pose.position.y = -0.4;
  GoalIntentExtractorWrapper extractor2;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber2 = nullptr;
  extractor2.configure(
    node, graph, &id_map, nullptr, costmap_subscriber2, "map", "map", "base_link");
  extractor2.setStartAndGoal(start, goal);
  rtn = extractor2.pruneStartandGoal(route, poses_goal, rerouting_info);
  EXPECT_EQ(rtn.edges.size(), 2u);
}
