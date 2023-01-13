// Copyright (c) 2023, Samsung Research America
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

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_route/route_planner.hpp"

namespace nav2_route
{

void RoutePlanner::configure(nav2_util::LifecycleNode::SharedPtr node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "max_iterations", rclcpp::ParameterValue(0));
  max_iterations_ = node->get_parameter("max_iterations").as_int();

  if (max_iterations_ == 0) {
    max_iterations_ = std::numeric_limits<int>::max();
  }

  edge_scorer_ = std::make_unique<EdgeScorer>(node);
}

Route RoutePlanner::findRoute(Graph & graph, unsigned int start, unsigned int goal)
{
  if (graph.empty()) {
    throw nav2_core::NoValidGraph("Graph is invalid for routing!");
  }

  // Find the start and goal pointers, it is important in this function
  // that these are the actual pointers, so that copied addresses are
  // not lost in the route when this function goes out of scope.
  const NodePtr & start_node = &graph.at(start);
  const NodePtr & goal_node = &graph.at(goal);
  findShortestGraphTraversal(graph, start_node, goal_node);

  EdgePtr & parent_edge = goal_node->search_state.parent_edge;
  if (!parent_edge) {
    throw nav2_core::NoValidRouteCouldBeFound("Could not find a route to the requested goal!");
  }

  // Convert graph traversal into a meaningful route
  Route route;
  while (parent_edge) {
    route.edges.push_back(parent_edge);
    parent_edge = parent_edge->start->search_state.parent_edge;
  }

  if (!parent_edge->start || parent_edge->start->nodeid != start_node->nodeid) {
    throw nav2_core::NoValidRouteCouldBeFound("Could not find a valid route!");
  }

  std::reverse(route.edges.begin(), route.edges.end());
  route.start_node = start_node;
  route.route_cost = goal_node->search_state.cost;
  return route;
}

void RoutePlanner::resetSearchStates(Graph & graph)
{
  // For graphs < 75,000 nodes, iterating through one time on initialization to reset the state
  // is neglibably different to allocating & deallocating the complimentary blocks of memory
  for (unsigned int i = 0; i != graph.size(); i++) {
    graph[i].search_state.reset();
  }
}

void RoutePlanner::findShortestGraphTraversal(
  Graph & graph, const NodePtr start,
  const NodePtr goal)
{
  // Setup the Dijkstra's search problem
  goal_id_ = goal->nodeid;
  resetSearchStates(graph);
  start->search_state.cost = 0.0;
  addNode(0.0, start);

  NodePtr neighbor{nullptr};
  EdgePtr edge{nullptr};
  float potential_cost = 0.0;
  int iterations = 0;
  while (!queue_.empty() && iterations < max_iterations_) {
    iterations++;

    // Get the next lowest cost node
    auto [curr_cost, node] = getNextNode();

    // This has been visited, thus already lowest cost
    if (curr_cost != node->search_state.cost) {
      continue;
    }

    // We have the shortest path
    if (isGoal(node)) {
      break;
    }

    // Expand to connected nodes
    EdgeVector & edges = getEdges(node);
    for (unsigned int edge_num = 0; edge_num != edges.size(); edge_num++) {
      edge = &edges[edge_num];
      neighbor = edge->end;
      potential_cost = curr_cost + getTraversalCost(edge);
      if (potential_cost < neighbor->search_state.cost) {
        neighbor->search_state.parent_edge = edge;
        neighbor->search_state.cost = potential_cost;
        addNode(potential_cost, neighbor);
      }
    }
  }

  // Reset state
  clearQueue();

  if (iterations == max_iterations_) {
    throw nav2_core::TimedOut("Maximum iterations was exceeded!");
  }
}

float RoutePlanner::getTraversalCost(const EdgePtr edge)
{
  if (!edge->edge_cost.overridable || edge_scorer_->numPlugins() == 0) {
    if (edge->edge_cost.cost != 0.0) {
      return edge->edge_cost.cost;
    } else {
      // We need some non-zero value if users didn't populate their files properly
      return hypotf(
        edge->end->coords.x - edge->start->coords.x,
        edge->end->coords.y - edge->start->coords.y);
    }
  }

  return edge_scorer_->score(edge);
}

NodeElement RoutePlanner::getNextNode()
{
  NodeElement data = queue_.top();
  queue_.pop();
  return data;
}

void RoutePlanner::addNode(const float cost, const NodePtr node)
{
  queue_.emplace(cost, node);
}

EdgeVector & RoutePlanner::getEdges(const NodePtr node)
{
  return node->neighbors;
}

void RoutePlanner::clearQueue()
{
  // TODO test if just reconstructing each request, or while pop, or this is fastest
  NodeQueue q;
  std::swap(queue_, q);
}

bool RoutePlanner::isGoal(const NodePtr node)
{
  return node->nodeid == goal_id_;
}

}  // namespace nav2_route
