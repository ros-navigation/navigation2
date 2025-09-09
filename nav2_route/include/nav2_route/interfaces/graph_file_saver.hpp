// Copyright (c) 2024 John Chrosniak
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

#ifndef NAV2_ROUTE__INTERFACES__GRAPH_FILE_SAVER_HPP_
#define NAV2_ROUTE__INTERFACES__GRAPH_FILE_SAVER_HPP_

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"

namespace nav2_route
{

/**
 * @class GraphFileSaver
 * @brief A plugin interface to parse a file into the graph
 */
class GraphFileSaver
{
public:
  using Ptr = std::shared_ptr<GraphFileSaver>;

  /**
   * @brief Constructor
   */
  GraphFileSaver() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~GraphFileSaver() = default;

  /**
   * @brief Configure the graph file saver, but do not store the node
   * @param parent pointer to user's node
   */
  virtual void configure(
    const nav2_util::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief Method to save the graph to the filepath
   * @param graph The graph to save
   * @param filepath The path to save the file to
   * @return true if graph was successfully saved
   */
  virtual bool saveGraphToFile(
    Graph & graph,
    std::string filepath) = 0;
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__INTERFACES__GRAPH_FILE_SAVER_HPP_
