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
#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/precomputed_route.hpp"

namespace nav2_route
{

class PrecomputedRouteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    graph.resize(3);
    for (unsigned int i = 0; i < graph.size(); ++i) {
      graph[i].nodeid = i + 1;
    }
    EdgeCost cost{2.0f, false};
    graph[0].addEdge(cost, &graph[1], 10);
    graph[1].addEdge(cost, &graph[2], 20);
    graph[2].addEdge(cost, &graph[0], 30);
  }
  Graph graph;
};

TEST_F(PrecomputedRouteTest, PreservesCanonicalObjectsAndOperations)
{
  graph[0].metadata.setValue("fleet", std::string("test"));
  graph[0].neighbors[0].operations.push_back({"stop", OperationTrigger::ON_ENTER, {}});
  auto route = resolvePrecomputedRoute(graph, 1, {10, 20});
  EXPECT_EQ(route.start_node, &graph[0]);
  ASSERT_EQ(route.edges.size(), 2u);
  EXPECT_EQ(route.edges[0], &graph[0].neighbors[0]);
  EXPECT_EQ(route.edges[1], &graph[1].neighbors[0]);
  EXPECT_EQ(route.edges[0]->operations[0].type, "stop");
  EXPECT_FLOAT_EQ(route.route_cost, 4.0f);
}

TEST_F(PrecomputedRouteTest, EmptySequenceIsSingleNode)
{
  auto route = resolvePrecomputedRoute(graph, 2, {});
  EXPECT_EQ(route.start_node, &graph[1]);
  EXPECT_TRUE(route.edges.empty());
  EXPECT_FLOAT_EQ(route.route_cost, 0.0f);
}

TEST_F(PrecomputedRouteTest, PreservesLoopsAndRepeatedEdges)
{
  auto route = resolvePrecomputedRoute(graph, 1, {10, 20, 30, 10});
  ASSERT_EQ(route.edges.size(), 4u);
  EXPECT_EQ(route.edges.front(), route.edges.back());
}

TEST_F(PrecomputedRouteTest, RejectsEmptyGraph)
{
  Graph empty;
  EXPECT_THROW(resolvePrecomputedRoute(empty, 1, {}), nav2_core::NoValidGraph);
}

TEST_F(PrecomputedRouteTest, RejectsUnknownNode)
{
  EXPECT_THROW(resolvePrecomputedRoute(graph, 99, {}), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsUnknownEdge)
{
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {99}), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsDisconnectedAndReverseEdges)
{
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {20}), nav2_core::NoValidRouteCouldBeFound);
  EXPECT_THROW(resolvePrecomputedRoute(graph, 2, {10}), nav2_core::NoValidRouteCouldBeFound);
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {10, 30}), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsAmbiguousOutgoingEdges)
{
  EdgeCost cost;
  graph[0].addEdge(cost, &graph[2], 10);
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {10}), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, ResolvesPairedReverseIDsAtCurrentNode)
{
  EdgeCost cost;
  graph[1].addEdge(cost, &graph[0], 10);
  auto route = resolvePrecomputedRoute(graph, 1, {10, 10});
  ASSERT_EQ(route.edges.size(), 2u);
  EXPECT_EQ(route.edges.back()->end, &graph[0]);
}

TEST_F(PrecomputedRouteTest, RejectsAmbiguousNode)
{
  graph[2].nodeid = 1;
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {}), nav2_core::NoValidRouteCouldBeFound);
}

TEST_F(PrecomputedRouteTest, RejectsMalformedEndpoints)
{
  graph[0].neighbors[0].end = nullptr;
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {10}), nav2_core::NoValidRouteCouldBeFound);
  graph[0].neighbors[0].end = &graph[1];
  graph[0].neighbors[0].start = &graph[1];
  EXPECT_THROW(resolvePrecomputedRoute(graph, 1, {10}), nav2_core::NoValidRouteCouldBeFound);
}

}  // namespace nav2_route
