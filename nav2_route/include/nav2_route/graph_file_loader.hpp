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
#include "tf2_ros/transform_listener.h"
#include "nav2_util/node_utils.hpp"

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
    const std::string frame)
  {
    // TODO plugin header + loader to use instead. THis will manage plugin get / load
    // TODO logging
    tf_ = tf;
    route_frame_ = frame;
    (void)node;
    nav2_util::declare_parameter_if_not_declared(
      node, "graph_filepath", rclcpp::ParameterValue(std::string("hi!")));
    // TODO rclcpp::ParameterType::PARAMETER_STRING
    graph_filepath_ = node->get_parameter("graph_filepath").as_string();
  }

  /**
   * @brief A destructor for nav2_route::GraphFileLoader
   */
  ~GraphFileLoader() = default;

  /**
   * @brief Loads a graph object with file information
   * @param graph Graph to populate
   * @return bool If successful
   */
  bool loadGraphFromFile(Graph & graph, std::string filepath = "")
  {
    // Check filepath exists TODO
    std::string filepath_to_load;
    if (filepath.empty()) {
      filepath_to_load = graph_filepath_;
    } else {
      filepath_to_load = filepath;
    }

    // if (!fileExists(filepath_to_load)) {
    //   RCLCPP_ERROR(node->get_logger(), "Graph file %s does not exist!", graph_filename_);
    //   return false;
    // }

    // Validate file is legit using a plugin API TODO
    // Load file using a plugin API
    // Convert all coordinates to `frame` (in a new method) for standardization

    // A test graph for visualization and prototyping
    graph.resize(9);
    unsigned int idx = 0;
    unsigned int ids = 1;
    for (unsigned int i = 0; i != 3; i++) {
      for (unsigned int j = 0; j != 3; j++) {
        graph[idx].nodeid = ids;
        graph[idx].coords.x = i;
        graph[idx].coords.y = j;
        idx++;
        ids++;
      }
    }

    EdgeCost default_cost;
    // Creates bidirectional routs from 0-1-4-5 & directional only route from 0-3-6
    graph[0].addEdge(default_cost, &graph[1], ids++);
    graph[1].addEdge(default_cost, &graph[0], ids++);
    graph[4].addEdge(default_cost, &graph[1], ids++);
    graph[1].addEdge(default_cost, &graph[4], ids++);
    graph[5].addEdge(default_cost, &graph[4], ids++);
    graph[4].addEdge(default_cost, &graph[5], ids++);
    graph[0].addEdge(default_cost, &graph[3], ids++);
    graph[3].addEdge(default_cost, &graph[6], ids++);
    return true;
  };

  /**
   * @brief Checks if a file even exists on the filesystem
   * @param filepath file to check
   * @return bool If the file exists
   */
  bool fileExists(const std::string & filepath)
  {
    return std::filesystem::exists(filepath);
  }

protected:
  std::string route_frame_, graph_filepath_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__GRAPH_FILE_LOADER_HPP_
