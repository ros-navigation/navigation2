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
#include "nav2_route/utils.hpp"
#include "nav2_route/types.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

TEST(TypesTest, test_metadata)
{
  Metadata mdata;
  float flt = 0.8f;
  std::string str = "value";
  unsigned int uintv = 17u;
  mdata.setValue<std::string>("key", str);
  mdata.setValue<float>("speed_limit", flt);
  mdata.setValue<unsigned int>("graph_id", uintv);

  float default_flt = 1.0f;
  std::string default_str = "";
  unsigned int default_uint = 0u;
  EXPECT_EQ(mdata.getValue<std::string>("key", default_str), str);
  EXPECT_EQ(mdata.getValue<float>("speed_limit", default_flt), flt);
  EXPECT_EQ(mdata.getValue<unsigned int>("graph_id", default_uint), uintv);
}

TEST(TypesTest, test_search_state)
{
  DirectionalEdge edge;
  SearchState state;
  state.parent_edge = &edge;
  state.integrated_cost = 25;
  state.traversal_cost = 250;

  state.reset();
  EXPECT_EQ(state.integrated_cost, std::numeric_limits<float>::max());
  EXPECT_EQ(state.traversal_cost, std::numeric_limits<float>::max());
  EXPECT_EQ(state.parent_edge, nullptr);
}

TEST(TypesTest, test_node)
{
  Node node1, node2;
  node1.nodeid = 50u;
  node2.nodeid = 51u;

  EdgeCost cost;
  cost.overridable = false;
  cost.cost = 100.0;
  EXPECT_EQ(node1.neighbors.size(), 0u);
  node1.addEdge(cost, &node2, 52u);

  EXPECT_EQ(node1.neighbors.size(), 1u);
  EXPECT_EQ(node1.neighbors[0].edgeid, 52u);
  EXPECT_EQ(node1.neighbors[0].start->nodeid, node1.nodeid);
  EXPECT_EQ(node1.neighbors[0].end->nodeid, node2.nodeid);
  EXPECT_EQ(node1.neighbors[0].edge_cost.cost, 100.0);
  EXPECT_FALSE(node1.neighbors[0].edge_cost.overridable);
}

TEST(UtilsTest, test_to_msg_conversions)
{
  // Test conversion of PoseStamped
  auto pose_msg = utils::toMsg(50.0, 20.0);
  EXPECT_EQ(pose_msg.pose.position.x, 50.0);
  EXPECT_EQ(pose_msg.pose.position.y, 20.0);

  // Test conversion of Route
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 10;
  test_node2.nodeid = 11;
  test_node3.nodeid = 12;

  DirectionalEdge test_edge1, test_edge2;
  test_edge1.edgeid = 13;
  test_edge1.start = &test_node1;
  test_edge1.end = &test_node2;
  test_edge2.edgeid = 14;
  test_edge2.start = &test_node2;
  test_edge2.end = &test_node3;

  Route route;
  route.start_node = &test_node1;
  route.route_cost = 50.0;
  route.edges.push_back(&test_edge1);
  route.edges.push_back(&test_edge2);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);
  auto route_msg = utils::toMsg(route, frame, time);
  EXPECT_EQ(route_msg.header.frame_id, frame);
  EXPECT_EQ(route_msg.header.stamp.nanosec, time.nanoseconds());
  EXPECT_EQ(route_msg.route_cost, 50.0);

  EXPECT_EQ(route_msg.nodes.size(), 3u);
  EXPECT_EQ(route_msg.edges.size(), 2u);

  EXPECT_EQ(route_msg.nodes[0].nodeid, test_node1.nodeid);
  EXPECT_EQ(route_msg.nodes[1].nodeid, test_node2.nodeid);
  EXPECT_EQ(route_msg.nodes[2].nodeid, test_node3.nodeid);
  EXPECT_EQ(route_msg.edges[0].edgeid, test_edge1.edgeid);
  EXPECT_EQ(route_msg.edges[1].edgeid, test_edge2.edgeid);
}

TEST(UtilsTest, test_to_visualization_msg_conversion)
{
  // Test conversion of Route Graph as MarkerArray
  std::string frame = "fake_frame";
  rclcpp::Time time(1000);
  Graph graph;
  graph.resize(9);
  unsigned int idx = 0;
  unsigned int ids = 1;
  for (unsigned int i = 0; i != 3; i++) {
    for (unsigned int j = 0; j != 3; j++) {
      graph[idx].nodeid = ids;
      graph[idx].coords.x = i;
      graph[idx].coords.y = j;
      idx++;
      ids++;
    }
  }

  EdgeCost default_cost;
  graph[0].addEdge(default_cost, &graph[1], ids++);
  graph[1].addEdge(default_cost, &graph[0], ids++);
  graph[4].addEdge(default_cost, &graph[1], ids++);
  graph[1].addEdge(default_cost, &graph[4], ids++);
  graph[5].addEdge(default_cost, &graph[4], ids++);
  graph[4].addEdge(default_cost, &graph[5], ids++);
  graph[0].addEdge(default_cost, &graph[3], ids++);
  graph[3].addEdge(default_cost, &graph[6], ids++);

  auto graph_msg = utils::toMsg(graph, frame, time);
  constexpr size_t expected_edge_markers = 1;
  constexpr size_t expected_node_markers = 1;
  constexpr size_t expected_edge_id_text_markers = 8;
  constexpr size_t expected_node_id_text_markers = 9;
  constexpr size_t expected_total_markers =
    expected_edge_markers + expected_node_markers +
    expected_edge_id_text_markers + expected_node_id_text_markers;

  EXPECT_EQ(graph_msg.markers.size(), expected_total_markers);
  for (auto & marker : graph_msg.markers) {
    if (marker.ns == "route_graph_ids") {
      EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    } else if (marker.ns == "route_graph") {
      EXPECT_TRUE(
        (marker.type == visualization_msgs::msg::Marker::LINE_LIST) ||
        (marker.type == visualization_msgs::msg::Marker::SPHERE));
    }
  }
}

TEST(UtilsTest, test_normalized_dot)
{
  // Vectors are orthogonal
  float v1x = 0;
  float v1y = 1;
  float v2x = 1;
  float v2y = 0;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), 0.0, 1e-4);

  // Vectors are identical
  v2x = 0;
  v2y = 1;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), 1.0, 1e-4);

  // vectors are opposite direction
  v2x = 0;
  v2y = -1;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), -1.0, 1e-4);

  // One vector is null
  v2x = 0;
  v2y = 0;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), 0.0, 1e-4);

  // Both are null
  v1x = 0;
  v1y = 0;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), 0.0, 1e-4);

  // Try un-normalized overlap / opposite / orthogonal
  v1x = 10;
  v1y = 0;
  v2x = 0;
  v2y = 6;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), 0.0, 1e-4);

  v2x = 4.5;
  v2y = 0;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), 1.0, 1e-4);

  v2x = -4.5;
  v2y = 0;
  EXPECT_NEAR(utils::normalizedDot(v1x, v1y, v2x, v2y), -1.0, 1e-4);
}

TEST(UtilsTest, test_find_closest_point)
{
  geometry_msgs::msg::PoseStamped pose;
  Coordinates start, end;
  start.x = 0.0;
  start.y = 0.0;
  end.x = 10.0;
  end.y = 0.0;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;

  // Test at, far from, and away from initial point
  Coordinates rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  pose.pose.position.x = -10.0;
  pose.pose.position.y = 0.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  pose.pose.position.x = -10.0;
  pose.pose.position.y = 100.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  // Test at, far from, and away from final point
  pose.pose.position.x = 10.0;
  pose.pose.position.y = 0.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 10.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  pose.pose.position.x = 100.0;
  pose.pose.position.y = 0.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 10.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  pose.pose.position.x = 100.0;
  pose.pose.position.y = -10000.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 10.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  pose.pose.position.x = 1000.0;
  pose.pose.position.y = 1000.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 10.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  // Test along length of the line
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 1000.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 5.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  pose.pose.position.x = 0.1;
  pose.pose.position.y = -10.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 0.1, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  // Let's try a more legit line now that we know the basics work OK
  start.x = 0.0;
  start.y = 0.0;
  end.x = 10.0;
  end.y = 10.0;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 5.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 5.0, 0.01);
  EXPECT_NEAR(rtn.y, 5.0, 0.01);

  pose.pose.position.x = 0.0;
  pose.pose.position.y = 10.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 5.0, 0.01);
  EXPECT_NEAR(rtn.y, 5.0, 0.01);

  pose.pose.position.x = 10.0;
  pose.pose.position.y = 0.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 5.0, 0.01);
  EXPECT_NEAR(rtn.y, 5.0, 0.01);

  pose.pose.position.x = 2.0;
  pose.pose.position.y = 4.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 3.0, 0.01);
  EXPECT_NEAR(rtn.y, 3.0, 0.01);

  pose.pose.position.x = 4.0;
  pose.pose.position.y = 10.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 7.0, 0.01);
  EXPECT_NEAR(rtn.y, 7.0, 0.01);

  // Try identity to make sure no nan issues
  start.x = 0.0;
  start.y = 0.0;
  end.x = 0.0;
  end.y = 0.0;
  pose.pose.position.x = 4.0;
  pose.pose.position.y = 10.0;
  rtn = utils::findClosestPoint(pose, start, end);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);
}

TEST(UtilsTest, test_routing_state)
{
  ReroutingState state;
  state.blocked_ids.resize(10);
  state.first_time = false;
  state.closest_pt_on_edge.x = 1.0;
  state.rerouting_start_id = 10u;
  state.rerouting_start_pose.pose.position.x = 1.0;
  state.reset();
  EXPECT_EQ(state.blocked_ids.size(), 0u);
  EXPECT_EQ(state.first_time, true);
  EXPECT_EQ(state.closest_pt_on_edge.x, 0.0);
  EXPECT_EQ(state.rerouting_start_id, std::numeric_limits<unsigned int>::max());
  EXPECT_EQ(state.rerouting_start_pose.pose.position.x, 0.0);
}
