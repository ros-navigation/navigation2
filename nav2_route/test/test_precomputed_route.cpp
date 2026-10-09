// Copyright (c) 2026 Yong Ling
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
#include <vector>

#include "gtest/gtest.h"
#include "nav2_route/route_server.hpp"

namespace nav2_route
{

class ResolvableRouteServer : public RouteServer
{
public:
  using RouteServer::resolveRoute;
  using RouteServer::graph_;
  using RouteServer::id_to_graph_map_;
};

class PrecomputedRouteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server = std::make_shared<ResolvableRouteServer>();
    server->graph_.resize(3);
    for (unsigned int i = 0; i < server->graph_.size(); ++i) {
      server->graph_[i].nodeid = i + 1;
      server->id_to_graph_map_[i + 1] = i;
    }
    EdgeCost cost{2.0f, false};
    auto & graph = server->graph_;
    graph[0].addEdge(cost, &graph[1], 10);
    graph[1].addEdge(cost, &graph[2], 20);
    graph[2].addEdge(cost, &graph[0], 30);
  }

  nav2_msgs::msg::Route message(
    const std::vector<uint16_t> & nodes, const std::vector<uint16_t> & edges)
  {
    nav2_msgs::msg::Route msg;
    for (auto id : nodes) {
      nav2_msgs::msg::RouteNode node;
      node.nodeid = id;
      msg.nodes.push_back(node);
    }
    for (auto id : edges) {
      nav2_msgs::msg::RouteEdge edge;
      edge.edgeid = id;
      msg.edges.push_back(edge);
    }
    return msg;
  }

  std::shared_ptr<ResolvableRouteServer> server;
};

TEST_F(PrecomputedRouteTest, PreservesCanonicalObjectsAndOperations)
{
  auto & graph = server->graph_;
  graph[0].coords.x = 4.0;
  graph[0].metadata.setValue("fleet", std::string("test"));
  graph[0].neighbors[0].operations.push_back({"stop", OperationTrigger::ON_ENTER, {}});
  auto msg = message({1, 2, 3}, {10, 20});
  msg.nodes[0].position.x = 999.0;
  msg.route_cost = 999.0;
  auto route = server->resolveRoute(msg);
  EXPECT_EQ(route.start_node, &graph[0]);
  EXPECT_EQ(route.start_node->metadata.getValue<std::string>("fleet", ""), "test");
  EXPECT_FLOAT_EQ(route.start_node->coords.x, 4.0f);
  ASSERT_EQ(route.edges.size(), 2u);
  EXPECT_EQ(route.edges[0], &graph[0].neighbors[0]);
  EXPECT_EQ(route.edges[1], &graph[1].neighbors[0]);
  EXPECT_EQ(route.edges[0]->operations[0].type, "stop");
  EXPECT_FLOAT_EQ(route.route_cost, 4.0f);
}

TEST_F(PrecomputedRouteTest, SingleNodeRoute)
{
  auto route = server->resolveRoute(message({2}, {}));
  EXPECT_EQ(route.start_node, &server->graph_[1]);
  EXPECT_TRUE(route.edges.empty());
}

TEST_F(PrecomputedRouteTest, PreservesLoopsAndRepeatedEdges)
{
  auto route = server->resolveRoute(message({1, 2, 3, 1, 2}, {10, 20, 30, 10}));
  ASSERT_EQ(route.edges.size(), 4u);
  EXPECT_EQ(route.edges.front(), route.edges.back());
}

TEST_F(PrecomputedRouteTest, RejectsEmptyGraph)
{
  server->graph_.clear();
  EXPECT_THROW(server->resolveRoute(message({1}, {})), nav2_core::NoValidGraph);
}

TEST_F(PrecomputedRouteTest, RejectsEmptyAndInconsistentSequences)
{
  for (const auto & msg : {message({}, {}), message({1, 2}, {}), message({1}, {10})}) {
    EXPECT_THROW(server->resolveRoute(msg), nav2_core::NoValidRouteCouldBeFound);
  }
}

TEST_F(PrecomputedRouteTest, RejectsUnknownNodeAndEdge)
{
  EXPECT_THROW(server->resolveRoute(message({99}, {})), nav2_core::NoValidRouteCouldBeFound);
  EXPECT_THROW(server->resolveRoute(message({1, 99}, {10})), nav2_core::NoValidRouteCouldBeFound);
  EXPECT_THROW(server->resolveRoute(message({1, 2}, {99})), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsDisconnectedAndReverseEdges)
{
  EXPECT_THROW(server->resolveRoute(message({1, 2}, {20})), nav2_core::NoValidRouteCouldBeFound);
  EXPECT_THROW(server->resolveRoute(message({2, 1}, {10})), nav2_core::NoValidRouteCouldBeFound);
  EXPECT_THROW(server->resolveRoute(message({1, 3}, {10})), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsAmbiguousDirectedEdge)
{
  EdgeCost cost;
  server->graph_[0].addEdge(cost, &server->graph_[1], 10);
  EXPECT_THROW(server->resolveRoute(message({1, 2}, {10})), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, UsesNextNodeToResolveReusedOutgoingID)
{
  EdgeCost cost;
  server->graph_[0].addEdge(cost, &server->graph_[2], 10);
  auto route = server->resolveRoute(message({1, 3}, {10}));
  EXPECT_EQ(route.edges.front(), &server->graph_[0].neighbors[1]);
}

TEST_F(PrecomputedRouteTest, ResolvesPairedReverseIDsAtCurrentNode)
{
  EdgeCost cost;
  server->graph_[1].addEdge(cost, &server->graph_[0], 10);
  auto route = server->resolveRoute(message({1, 2, 1}, {10, 10}));
  ASSERT_EQ(route.edges.size(), 2u);
  EXPECT_EQ(route.edges.back()->end, &server->graph_[0]);
}

TEST_F(PrecomputedRouteTest, RejectsMalformedEndpoints)
{
  server->graph_[0].neighbors[0].end = nullptr;
  EXPECT_THROW(server->resolveRoute(message({1, 2}, {10})), nav2_core::NoValidRouteCouldBeFound);
  server->graph_[0].neighbors[0].end = &server->graph_[1];
  server->graph_[0].neighbors[0].start = &server->graph_[1];
  EXPECT_THROW(server->resolveRoute(message({1, 2}, {10})), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsDifferentGraphFrame)
{
  auto msg = message({1}, {});
  msg.header.frame_id = "other_map";
  EXPECT_THROW(server->resolveRoute(msg), nav2_core::NoValidRouteCouldBeFound);
}

}  // namespace nav2_route

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int status = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return status;
}
