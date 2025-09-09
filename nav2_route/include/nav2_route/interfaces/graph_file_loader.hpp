// Copyright (c) 2025 Joshua Wallace
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

#ifndef NAV2_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_
#define NAV2_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"

namespace nav2_route
{

/**
 * @class GraphFileLoader
 * @brief A plugin interface to parse a file into the graph
 */
class GraphFileLoader
{
public:
  using Ptr = std::shared_ptr<GraphFileLoader>;

  /**
   * @brief Constructor
   */
  GraphFileLoader() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~GraphFileLoader() = default;

  /**
   * @brief Configure the graph file loader, but do not store the node
   * @param parent pointer to user's node
   */
  virtual void configure(
    const nav2_util::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief Method to load the graph from the filepath
   * @param graph The graph to populate
   * @param filepath The file to parse
   * @param idx_map A map translating nodeid's to graph idxs for use in graph modification
   * services and idx-based route planning requests. This is much faster than using a
   * map the full graph data structure.
   * @return true if graph was successfully loaded
   */
  virtual bool loadGraphFromFile(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    std::string filepath) = 0;
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_
