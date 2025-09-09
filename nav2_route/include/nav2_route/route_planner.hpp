// Copyright (c) 2025 Open Navigation LLC
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

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/edge_scorer.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"

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
  RoutePlanner() = default;

  /**
   * @brief A destructor for nav2_route::RoutePlanner
   */
  virtual ~RoutePlanner() = default;

  /**
   * @brief Configure the route planner, get parameters
   * @param node Node object to get parametersfrom
   * @param tf_buffer TF buffer to use for transformations
   * @param costmap_subscriber Costmap subscriber to use for blocked nodes
   */
  void configure(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber);

  /**
   * @brief Find the route from start to goal on the graph
   * @param graph Graph to search
   * @param start Start index in the graph of the start node
   * @param goal Goal index in the graph of the goal node
   * @param blocked_ids A set of blocked node and edge IDs not to traverse
   * @return Route object containing the navigation graph route
   */
  virtual Route findRoute(
    Graph & graph, unsigned int start_index, unsigned int goal_index,
    const std::vector<unsigned int> & blocked_ids,
    const RouteRequest & route_request);

protected:
  /**
   * @brief Reset the search state of the graph nodes
   * @param graph Graph to reset
   */
  inline void resetSearchStates(Graph & graph);

  /**
   * @brief Dikstra's algorithm search on the graph
   * @param graph Graph to search
   * @param start Start Node pointer
   * @param goal Goal node pointer
   * @param blocked_ids A set of blocked node and edge IDs not to traverse
   */
  void findShortestGraphTraversal(
    Graph & graph, const NodePtr start_node, const NodePtr goal_node,
    const std::vector<unsigned int> & blocked_ids,
    const RouteRequest & route_request);

  /**
   * @brief Gets the traversal cost for an edge using edge scorers
   * @param edge Edge pointer to find traversal cost for
   * @param travel cost
   * @param blocked_ids A set of blocked node and edge IDs not to traverse
   * @return if this edge is valid for search
   */
  inline bool getTraversalCost(
    const EdgePtr edge, float & score, const std::vector<unsigned int> & blocked_ids,
    const RouteRequest & route_request);

  /**
   * @brief Gets the next node in the priority queue for search
   * @return Next node pointer in queue with cost
   */
  inline NodeElement getNextNode();

  /**
   * @brief Adds a node to the priority queue for search
   * @param cost Priority level
   * @param node Node pointer to insert
   */
  inline void addNode(const float cost, const NodePtr node);

  /**
   * @brief Gets the edges from a given node
   * @param node Node pointer to check
   * @return A vector of edges that the node contains
   */
  inline EdgeVector & getEdges(const NodePtr node);

  /**
   * @brief Clears the priority queue
   */
  inline void clearQueue();

  /**
   * @brief Checks if a given node is the goal node
   * @param node Node to check
   * @return bool If this node is the goal
   */
  inline bool isGoal(const NodePtr node);

  /**
   * @brief Checks if a given node is the start node
   * @param node Node to check
   * @return bool If this node is the start
   */
  inline bool isStart(const NodePtr node);

  /**
   * @brief Checks edge is a start or end edge
   * @param edge Edge to check
   * @return EdgeType identifying whether the edge is start, end or none
   */
  nav2_route::EdgeType classifyEdge(const EdgePtr edge);

  int max_iterations_{0};
  unsigned int start_id_{0};
  unsigned int goal_id_{0};
  NodeQueue queue_;
  std::unique_ptr<EdgeScorer> edge_scorer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__ROUTE_PLANNER_HPP_
