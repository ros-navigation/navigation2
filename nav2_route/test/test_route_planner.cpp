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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/route_planner.hpp"
#include "nav2_msgs/action/compute_route.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

inline Graph create4x4Graph()
{
  // * - * - * > *    12 13 14 15
  // |   |   |   ^
  // * - * - * - *    8  9  10 11
  // |   |   |   |
  // * - * - * - *    4  5   6  7
  // |   |   |   |
  // * - * - * - *    0  1   2  3
  // where `-` and `|` are bidirectional edges and > are single direction edges

  EdgeCost default_cost;
  Graph graph;
  graph.resize(16);
  unsigned int idx = 1;
  for (unsigned int j = 0; j != 4; j++) {
    for (unsigned int i = 0; i != 4; i++) {
      Node & node = graph[idx - 1];
      node.coords.x = i;
      node.coords.y = j;
      node.nodeid = idx;
      idx++;
    }
  }

  // Bottom row
  graph[0].addEdge(default_cost, &graph[1], idx++);
  graph[1].addEdge(default_cost, &graph[0], idx++);
  graph[0].addEdge(default_cost, &graph[4], idx++);
  graph[4].addEdge(default_cost, &graph[0], idx++);

  graph[1].addEdge(default_cost, &graph[2], idx++);
  graph[2].addEdge(default_cost, &graph[1], idx++);
  graph[1].addEdge(default_cost, &graph[5], idx++);
  graph[5].addEdge(default_cost, &graph[1], idx++);

  graph[2].addEdge(default_cost, &graph[3], idx++);
  graph[3].addEdge(default_cost, &graph[2], idx++);
  graph[2].addEdge(default_cost, &graph[6], idx++);
  graph[6].addEdge(default_cost, &graph[2], idx++);

  graph[7].addEdge(default_cost, &graph[3], idx++);
  graph[3].addEdge(default_cost, &graph[7], idx++);

  // Second row
  graph[4].addEdge(default_cost, &graph[5], idx++);
  graph[5].addEdge(default_cost, &graph[4], idx++);
  graph[4].addEdge(default_cost, &graph[8], idx++);
  graph[8].addEdge(default_cost, &graph[4], idx++);

  graph[5].addEdge(default_cost, &graph[6], idx++);
  graph[6].addEdge(default_cost, &graph[5], idx++);
  graph[5].addEdge(default_cost, &graph[9], idx++);
  graph[9].addEdge(default_cost, &graph[5], idx++);

  graph[6].addEdge(default_cost, &graph[7], idx++);
  graph[7].addEdge(default_cost, &graph[6], idx++);
  graph[6].addEdge(default_cost, &graph[10], idx++);
  graph[10].addEdge(default_cost, &graph[6], idx++);

  graph[7].addEdge(default_cost, &graph[11], idx++);
  graph[11].addEdge(default_cost, &graph[7], idx++);

  // third row
  graph[8].addEdge(default_cost, &graph[9], idx++);
  graph[9].addEdge(default_cost, &graph[8], idx++);
  graph[8].addEdge(default_cost, &graph[12], idx++);
  graph[12].addEdge(default_cost, &graph[8], idx++);

  graph[9].addEdge(default_cost, &graph[10], idx++);
  graph[10].addEdge(default_cost, &graph[9], idx++);
  graph[9].addEdge(default_cost, &graph[13], idx++);
  graph[13].addEdge(default_cost, &graph[9], idx++);

  graph[10].addEdge(default_cost, &graph[11], idx++);
  graph[11].addEdge(default_cost, &graph[10], idx++);
  graph[10].addEdge(default_cost, &graph[14], idx++);
  graph[14].addEdge(default_cost, &graph[10], idx++);

  graph[11].addEdge(default_cost, &graph[15], idx++);  // one direction

  // Top row
  graph[12].addEdge(default_cost, &graph[13], idx++);
  graph[13].addEdge(default_cost, &graph[12], idx++);

  graph[13].addEdge(default_cost, &graph[14], idx++);
  graph[14].addEdge(default_cost, &graph[13], idx++);

  graph[14].addEdge(default_cost, &graph[15], idx++);  // one direction
  return graph;
}

TEST(RoutePlannerTest, test_route_planner_positive)
{
  geometry_msgs::msg::PoseStamped start_pose, goal_pose;
  RouteRequest route_request;

  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> collision_checker;
  RoutePlanner planner;
  planner.configure(node, tf_buffer, collision_checker);
  std::vector<unsigned int> blocked_ids;
  unsigned int start, goal;

  // Create a graph to test routing upon.
  Graph graph = create4x4Graph();

  // Plan across diagonal, should be length 6
  start = 0u;
  goal = 15u;
  Route route = planner.findRoute(graph, start, goal, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 6.0, 0.001);
  EXPECT_EQ(route.edges.size(), 6u);

  // If we try in reverse, it should fail though since Node 16 is only
  // achievable from the other direction
  start = 15;
  goal = 0;
  EXPECT_THROW(
    planner.findRoute(
      graph, start, goal, blocked_ids,
      route_request), nav2_core::NoValidRouteCouldBeFound);

  // Let's find a simple plan and then mess with the planner with blocking edges
  start = 0;
  goal = 12;
  route = planner.findRoute(graph, start, goal, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 3.0, 0.001);
  EXPECT_EQ(route.edges.size(), 3u);

  // Now block an edge as closed along the chain, should find the next best path
  unsigned int key_edgeid = 19u;
  blocked_ids.push_back(key_edgeid);  // Edge between node 0-4 in the 0-4-9-12 chain
  route = planner.findRoute(graph, start, goal, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 5.0, 0.001);
  EXPECT_EQ(route.edges.size(), 5u);
  for (auto & edge : route.edges) {
    EXPECT_NE(edge->edgeid, key_edgeid);
  }

  // Now don't block, but instead just increase the cost so that it goes elsewhere
  // this should produce the same results
  blocked_ids.clear();
  graph[0].neighbors[1].edge_cost.cost = 1e6;
  graph[0].neighbors[1].edge_cost.overridable = false;
  route = planner.findRoute(graph, start, goal, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 5.0, 0.001);
  EXPECT_EQ(route.edges.size(), 5u);
}

TEST(RoutePlannerTest, test_route_planner_negative)
{
  geometry_msgs::msg::PoseStamped start_pose, goal_pose;
  RouteRequest route_request;

  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  node->declare_parameter("max_iterations", rclcpp::ParameterValue(5));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> collision_checker;
  RoutePlanner planner;
  planner.configure(node, tf_buffer, collision_checker);
  std::vector<unsigned int> blocked_ids;
  unsigned int start = 0;
  unsigned int goal = 15;
  Graph graph;

  // No graph yet, should fail
  EXPECT_THROW(
    planner.findRoute(
      graph, start, goal, blocked_ids,
      route_request), nav2_core::NoValidGraph);

  // Create a graph to test routing upon.
  graph = create4x4Graph();

  // Try with a stupidly low number of iterations
  graph[0].neighbors[1].edge_cost.overridable = true;
  EXPECT_THROW(
    planner.findRoute(
      graph, start, goal, blocked_ids,
      route_request), nav2_core::TimedOut);

  // If we try to plan but we both cannot override and has 0 cost, will throw
  graph[0].neighbors[1].edge_cost.overridable = false;
  graph[0].neighbors[1].edge_cost.cost = 0.0;
  EXPECT_THROW(
    planner.findRoute(
      graph, start, goal, blocked_ids,
      route_request), nav2_core::NoValidGraph);
}
