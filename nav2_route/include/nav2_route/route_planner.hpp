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

#ifndef NAV2_ROUTE__ROUTE_PLANNER_HPP_
#define NAV2_ROUTE__ROUTE_PLANNER_HPP_

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"

namespace nav2_route
{
/**
 * @class nav2_route::RoutePlanner
 * @brief An optimal planner to compute a route from a start to a goal in an arbitrary graph
 */
class RoutePlanner
{
public:
  /**
   * @brief A constructor for nav2_route::RoutePlanner
   */
  explicit RoutePlanner() = default;

  /**
   * @brief A destructor for nav2_route::RoutePlanner
   */
  ~RoutePlanner() = default;

  void configure(nav2_util::LifecycleNode::SharedPtr /*node*/)
  {
    // TODO get params, create and initialze edge scorer object, max iterations

    if (max_iterations_ == 0) {
      max_iterations_ = std::numeric_limits<int>::max();
    }
  }

  Route findRoute(Graph & graph, unsigned int start, unsigned int goal)
  {
    // Find the start and goal pointers, it is important in this function
    // that these are the actual pointers, so that copied addresses are
    // not lost in the route when this function goes out of scope.
    const NodePtr & start_node = &graph.at(start);
    const NodePtr & goal_node = &graph.at(goal);
    findShortestGraphTraversal(graph, start_node, goal_node);

    EdgePtr & parent_edge = goal_node->search_state.parent_edge;
    if (!parent_edge) {
      // TODO exception / log failed to find route from start to goal
    }

    // Convert graph traversal into a meaningful route
    Route route;
    while (parent_edge) {
      route.edges.push_back(parent_edge);
      parent_edge = parent_edge->start->search_state.parent_edge;
    }

    if (!parent_edge->start || parent_edge->start->nodeid != start_node->nodeid) {
      // TODO excpetion failed to get path properly, log
    }

    std::reverse(route.edges.begin(), route.edges.end());
    route.start_node = start_node;
    route.route_cost = goal_node->search_state.cost;
    return route;
  }

protected:
  inline void resetSearchStates(Graph & graph)
  {
    // For graphs < 75,000 nodes, iterating through one time on initialization to reset the state
    // is neglibably different to allocating & deallocating the complimentary blocks of memory
    for (unsigned int i = 0; i != graph.size(); i++) {
      graph[i].search_state.reset();
    }
  }

  void findShortestGraphTraversal(Graph & graph, const NodePtr start, const NodePtr goal)
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
      // TODO excetpion max its exceeded / log
    }
  }

  inline float getTraversalCost(const EdgePtr edge)
  {
    if (!edge->edge_cost.overridable /*|| plugins.empty()*/) {
      return edge->edge_cost.cost;
    }

    // TODO scoring plugins, for now, just distance
    return hypotf(
      edge->end->coords.x - edge->start->coords.x,
      edge->end->coords.y - edge->start->coords.y);
  }

  inline NodeElement getNextNode()
  {
    NodeElement data = queue_.top();
    queue_.pop();
    return data;
  }

  inline void addNode(const float cost, const NodePtr node)
  {
     queue_.emplace(cost, node);
  }

  inline EdgeVector & getEdges(const NodePtr node)
  {
    return node->neighbors;
  }

  inline void clearQueue()
  {
    // TODO test if just reconstructing each request, or while pop, or this is fastest
    NodeQueue q;
    std::swap(queue_, q);
  }

  inline bool isGoal(const NodePtr node)
  {
    return node->nodeid == goal_id_;
  }

  int max_iterations_{0};
  unsigned int goal_id_{0};
  NodeQueue queue_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__ROUTE_PLANNER_HPP_
