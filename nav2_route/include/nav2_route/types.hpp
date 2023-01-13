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
    data.insert({key, value});
  }

  std::unordered_map<std::string, std::any> data;
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


struct Node;
typedef Node * NodePtr;
typedef std::vector<Node> NodeVector;
typedef NodeVector Graph;
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
  float cost{0.0};

  void reset()
  {
    cost = std::numeric_limits<float>::max();
    parent_edge = nullptr;
  }
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
  SearchState search_state;  // State maintained by route search algorithm

  void addEdge(EdgeCost & cost, NodePtr node, unsigned int edgeid)
  {
    neighbors.push_back({edgeid, this, node, cost, {}});
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

}  // namespace nav2_route

#endif  // NAV2_ROUTE__TYPES_HPP_
