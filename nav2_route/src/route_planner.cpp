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

void RoutePlanner::configure(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber)
{
  nav2_util::declare_parameter_if_not_declared(
    node, "max_iterations", rclcpp::ParameterValue(0));
  max_iterations_ = node->get_parameter("max_iterations").as_int();

  if (max_iterations_ <= 0) {
    max_iterations_ = std::numeric_limits<int>::max();
  }

  edge_scorer_ = std::make_unique<EdgeScorer>(node, tf_buffer, costmap_subscriber);
}

Route RoutePlanner::findRoute(
  Graph & graph, unsigned int start_index, unsigned int goal_index,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  if (graph.empty()) {
    throw nav2_core::NoValidGraph("Graph is invalid for routing!");
  }

  // Find the start and goal pointers, it is important in this function
  // that the start node is the underlying pointer, so that the address
  // is valid when this function goes out of scope
  const NodePtr & start_node = &graph.at(start_index);
  const NodePtr & goal_node = &graph.at(goal_index);
  findShortestGraphTraversal(graph, start_node, goal_node, blocked_ids, route_request);

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

  std::reverse(route.edges.begin(), route.edges.end());
  route.start_node = start_node;
  route.route_cost = goal_node->search_state.integrated_cost;
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
  Graph & graph, const NodePtr start_node, const NodePtr goal_node,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  // Setup the Dijkstra's search
  resetSearchStates(graph);
  start_id_ = start_node->nodeid;
  goal_id_ = goal_node->nodeid;
  start_node->search_state.integrated_cost = 0.0;
  addNode(0.0, start_node);

  NodePtr neighbor{nullptr};
  EdgePtr edge{nullptr};
  float potential_cost = 0.0, traversal_cost = 0.0;
  int iterations = 0;
  while (!queue_.empty() && iterations < max_iterations_) {
    iterations++;

    // Get the next lowest cost node
    auto [curr_cost, node] = getNextNode();

    // This has been visited, thus already lowest cost
    if (curr_cost != node->search_state.integrated_cost) {
      continue;
    }

    // We have the shortest path
    if (isGoal(node)) {
      // Reset states
      clearQueue();
      return;
    }

    // Expand to connected nodes
    EdgeVector & edges = getEdges(node);
    for (unsigned int edge_num = 0; edge_num != edges.size(); edge_num++) {
      edge = &edges[edge_num];
      neighbor = edge->end;

      // If edge is invalid (lane closed, occupied, etc), don't expand
      if (!getTraversalCost(edge, traversal_cost, blocked_ids, route_request)) {
        continue;
      }

      potential_cost = curr_cost + traversal_cost;
      if (potential_cost < neighbor->search_state.integrated_cost) {
        neighbor->search_state.parent_edge = edge;
        neighbor->search_state.integrated_cost = potential_cost;
        neighbor->search_state.traversal_cost = traversal_cost;
        addNode(potential_cost, neighbor);
      }
    }
  }

  if (iterations == max_iterations_) {
    // Reset states
    clearQueue();
    throw nav2_core::TimedOut("Maximum iterations was exceeded!");
  }
}

bool RoutePlanner::getTraversalCost(
  const EdgePtr edge, float & score, const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  // If edge or node is in the blocked list, don't expand
  auto is_blocked = std::find_if(
    blocked_ids.begin(), blocked_ids.end(),
    [&](unsigned int id) {return id == edge->edgeid || id == edge->end->nodeid;});
  if (is_blocked != blocked_ids.end()) {
    return false;
  }

  // If an edge's cost is marked as not to be overridden by scoring plugins
  // Or there are no scoring plugins, use the edge's cost, if it is valid (positive)
  if (!edge->edge_cost.overridable || edge_scorer_->numPlugins() == 0) {
    if (edge->edge_cost.cost <= 0.0) {
      throw nav2_core::NoValidGraph(
              "Edge " + std::to_string(edge->edgeid) +
              " doesn't contain and cannot compute a valid edge cost!");
    }
    score = edge->edge_cost.cost;
    return true;
  }

  return edge_scorer_->score(edge, route_request, classifyEdge(edge), score);
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
  NodeQueue q;
  std::swap(queue_, q);
}

bool RoutePlanner::isGoal(const NodePtr node)
{
  return node->nodeid == goal_id_;
}

bool RoutePlanner::isStart(const NodePtr node)
{
  return node->nodeid == start_id_;
}

nav2_route::EdgeType RoutePlanner::classifyEdge(const EdgePtr edge)
{
  if (isStart(edge->start)) {
    return EdgeType::START;
  } else if (isGoal(edge->end)) {
    return EdgeType::END;
  }
  return nav2_route::EdgeType::NONE;
}

}  // namespace nav2_route
