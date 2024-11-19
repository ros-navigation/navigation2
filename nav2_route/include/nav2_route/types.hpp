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

#include <queue>
#include <string>
#include <any>
#include <vector>
#include <unordered_map>
#include <utility>
#include <limits>
#include <geometry_msgs/msg/pose_stamped.hpp>

#ifndef NAV2_ROUTE__TYPES_HPP_
#define NAV2_ROUTE__TYPES_HPP_

namespace nav2_route
{

/**
 * @struct nav2_route::Metadata
 * @brief An object to store arbitrary metadata regarding nodes from the graph file
 */
struct Metadata
{
  Metadata() {}

  // For retrieving metadata at run-time via plugins
  template<typename T>
  T getValue(const std::string & key, T & default_val) const
  {
    auto it = data.find(key);
    if (it == data.end()) {
      return default_val;
    }
    return std::any_cast<T>(it->second);
  }

  // For populating metadata from file
  template<typename T>
  void setValue(const std::string & key, T & value)
  {
    data[key] = value;
  }

  std::unordered_map<std::string, std::any> data;
};

struct Node;
typedef Node * NodePtr;
typedef std::vector<Node> NodeVector;
typedef NodeVector Graph;
typedef std::unordered_map<unsigned int, unsigned int> GraphToIDMap;
typedef std::unordered_map<unsigned int, std::vector<unsigned int>> GraphToIncomingEdgesMap;
typedef std::vector<NodePtr> NodePtrVector;
typedef std::pair<float, NodePtr> NodeElement;
typedef std::pair<unsigned int, unsigned int> NodeExtents;

/**
 * @struct nav2_route::NodeComparator
 * @brief Node comparison for priority queue sorting
 */
struct NodeComparator
{
  bool operator()(const NodeElement & a, const NodeElement & b) const
  {
    return a.first > b.first;
  }
};

typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

/**
 * @struct nav2_route::EdgeCost
 * @brief An object to store edge cost or cost metadata for scoring
 */
struct EdgeCost
{
  float cost{0.0};
  bool overridable{true};  // If overridable, may use plugin edge cost scorers
};

/**
 * @enum nav2_route::OperationTrigger
 * @brief The triggering events for an operation
 */
enum class OperationTrigger
{
  NODE = 0,
  ON_ENTER = 1,
  ON_EXIT = 2
};

/**
 * @struct nav2_route::Operation
 * @brief An object to store operations to perform on events with types and metadata
 */
struct Operation
{
  std::string type;
  OperationTrigger trigger;
  Metadata metadata;
};

typedef std::vector<Operation> Operations;
typedef std::vector<Operation *> OperationPtrs;

/**
 * @struct nav2_route::OperationsResult
 * @brief Result information from the operations manager
 */
struct OperationsResult
{
  std::vector<std::string> operations_triggered;
  bool reroute{false};
  std::vector<unsigned int> blocked_ids;
};

/**
 * @struct nav2_route::DirectionalEdge
 * @brief An object representing edges between nodes
 */
struct DirectionalEdge
{
  unsigned int edgeid;     // Edge identifier
  NodePtr start{nullptr};  // Ptr to starting node of edge
  NodePtr end{nullptr};    // Ptr to ending node of edge
  EdgeCost edge_cost;      // Cost information associated with edge
  Metadata metadata;       // Any metadata stored in the graph file of interest
  Operations operations;   // Operations to perform related to the edge
};

typedef DirectionalEdge * EdgePtr;
typedef std::vector<DirectionalEdge> EdgeVector;
typedef std::vector<EdgePtr> EdgePtrVector;

/**
 * @struct nav2_route::SearchState
 * @brief An object to store state related to graph searching of nodes
 * This is an internal class users should not modify.
 */
struct SearchState
{
  EdgePtr parent_edge{nullptr};
  float integrated_cost{std::numeric_limits<float>::max()};
  float traversal_cost{std::numeric_limits<float>::max()};

  void reset()
  {
    integrated_cost = std::numeric_limits<float>::max();
    traversal_cost = std::numeric_limits<float>::max();
    parent_edge = nullptr;
  }
};

/**
 * @struct nav2_route::Coordinates
 * @brief An object to store Node coordinates in different frames
 */
struct Coordinates
{
  std::string frame_id{"map"};
  float x{0.0}, y{0.0};
};

/**
 * @struct nav2_route::Node
 * @brief An object to store the nodes in the graph file
 */
struct Node
{
  unsigned int nodeid;       // Node identifier
  Coordinates coords;        // Coordinates of node
  EdgeVector neighbors;      // Directed neighbors and edges of the node
  Metadata metadata;         // Any metadata stored in the graph file of interest
  Operations operations;     // Operations to perform related to the node
  SearchState search_state;  // State maintained by route search algorithm

  void addEdge(
    EdgeCost & cost, NodePtr node, unsigned int edgeid, Metadata meta_data = {},
    Operations operations_data = {})
  {
    neighbors.push_back({edgeid, this, node, cost, meta_data, operations_data});
  }
};

/**
 * @struct nav2_route::Route
 * @brief An ordered set of nodes and edges corresponding to the planned route
 */
struct Route
{
  NodePtr start_node;
  EdgePtrVector edges;
  float route_cost{0.0};
};

/**
 * @enum nav2_route::TrackerResult
 * @brief Return result of the route tracker to the main server for processing
 */
enum class TrackerResult
{
  INTERRUPTED = 0,
  REROUTE = 1,
  COMPLETED = 2
};

/**
 * @struct nav2_route::RouteTrackingState
 * @brief Current state management of route tracking class
 */
struct RouteTrackingState
{
  NodePtr last_node{nullptr}, next_node{nullptr};
  EdgePtr current_edge{nullptr};
  int route_edges_idx{-1};
  bool within_radius{false};
};

/**
 * @struct nav2_route::ReroutingState
 * @brief State shared to objects to communicate important rerouting data
 * to avoid rerouting over blocked edges, ensure reroute from the current
 * appropriate starting point along the route, and state of edges if pruned
 * for seeding the Tracker's state.
 */
struct ReroutingState
{
  unsigned int rerouting_start_id{std::numeric_limits<unsigned int>::max()};
  std::vector<unsigned int> blocked_ids;
  bool first_time{true};
  EdgePtr curr_edge{nullptr};
  Coordinates closest_pt_on_edge;
  geometry_msgs::msg::PoseStamped rerouting_start_pose;

  void reset()
  {
    rerouting_start_id = std::numeric_limits<unsigned int>::max();
    blocked_ids.clear();
    first_time = true;
    curr_edge = nullptr;
    closest_pt_on_edge = Coordinates();
    rerouting_start_pose = geometry_msgs::msg::PoseStamped();
  }
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__TYPES_HPP_
