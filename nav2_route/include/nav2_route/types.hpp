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
  // For retrieving metadata at run-time via plugins
  template <typename T>
  T getValue(const std::string & key, T & default_val) const
  {
    auto it = data.find(key);
    if (it == data.end()) {
      return default_val;
    }
    return std::any_cast<T>(it->second);
  }

  // For populating metadata from file
  template <typename T>
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
  std::string frame_id;
  float x, y, yaw;
};

/**
 * @struct nav2_route::NodeCost
 * @brief An object to store edge cost or cost metadata for scoring
 */
struct EdgeCost
{
  float cost{0.0};
  bool overridable{true};  // If overridable, may use plugin edge cost scorers
};

struct Node;
typedef Node * NodePtr;
typedef std::vector<Node> NodeVector;
typedef std::vector<NodePtr> NodePtrVector;

struct DirectionalEdge
{
  EdgeCost cost;
  NodePtr end{nullptr};
  unsigned int edgeid;
};

typedef std::vector<DirectionalEdge> EdgeVector;

/**
 * @struct nav2_route::Node
 * @brief An object to store the nodes in the graph file
 */
struct Node
{
  unsigned int nodeid;      // Node identifier
  Coordinates coords;       // Coordinates of node
  EdgeVector neighbors;     // Directed neighbors and edges of the node
  Metadata metadata;        // Any metadata stored in the graph file of interest

  // TODO down line, try to see if seperate edge vector is better for any reason & store ptr
    // e.g. editing over time by edgeID (since now would have to ierate through every node to find it vs a edge map)
    // but eges should know the node root/end so we can find via that and populate edge info that way. But node lookup still a problem
  void addEdge(EdgeCost & cost, NodePtr node, unsigned int edgeid)
  {
    neighbors.push_back({cost, node, edgeid});
  }
};

// TODO populating edges on nodes might be better suited if can do lookups based on Ids (map, or Ids to position?) 
typedef NodeVector Graph;

typedef std::pair<unsigned int, unsigned int> NodeExtents;

}  // namespace nav2_route

#endif  // NAV2_ROUTE__TYPES_HPP_
