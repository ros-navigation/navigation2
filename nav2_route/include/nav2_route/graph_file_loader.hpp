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

#ifndef NAV2_ROUTE__GRAPH_FILE_LOADER_HPP_
#define NAV2_ROUTE__GRAPH_FILE_LOADER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_route/types.hpp"

namespace nav2_route
{
/**
 * @class nav2_route::GraphFileLoader
 * @brief An action server to load graph files into the graph object for search and processing
 */
class GraphFileLoader
{
public:
  /**
   * @brief A constructor for nav2_route::GraphFileLoader
   * @param options Additional options to control creation of the node.
   */
  explicit GraphFileLoader(
    nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame);

  /**
   * @brief A destructor for nav2_route::GraphFileLoader
   */
  ~GraphFileLoader() = default;

  /**
   * @brief Loads a graph object with file information
   * @param graph Graph to populate
   * @return bool If successful
   */
  bool loadGraphFromFile(Graph & graph, std::string filepath = "");

  /**
   * @brief Checks if a file even exists on the filesystem
   * @param filepath file to check
   * @return bool If the file exists
   */
  inline bool fileExists(const std::string & filepath);

protected:
  std::string route_frame_, graph_filepath_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__GRAPH_FILE_LOADER_HPP_
