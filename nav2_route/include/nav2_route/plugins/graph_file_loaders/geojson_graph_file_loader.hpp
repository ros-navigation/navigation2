// Copyright (c) 2023 Joshua Wallace
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


#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "nav2_route/interfaces/graph_file_loader.hpp"

#ifndef NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__GEOJSON_GRAPH_FILE_LOADER_HPP_
#define NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__GEOJSON_GRAPH_FILE_LOADER_HPP_

namespace nav2_route
{

/**
 * @class nav2_route::GeoJsonGraphFileLoader
 * @brief A GraphFileLoader plugin to parse the geojson graph file
 */
class GeoJsonGraphFileLoader : public GraphFileLoader
{
public:
  using Json = nlohmann::json;

  /**
   * @brief Constructor
   */
  GeoJsonGraphFileLoader() = default;

  /**
   * @brief Destructor
   */
  ~GeoJsonGraphFileLoader() = default;

  /**
   * @brief Loads a graph object with file information
   * @param graph Graph to populate
   * @param graph_to_id_map A map of nodeid's to graph indexs
   * @param filepath The file to load
   * @return bool If successful
   */
  bool loadGraphFromFile(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    std::string filepath) override;

  /**
 * @brief Checks if a file even exists on the filesystem
 * @param filepath file to check
 * @return bool If the file exists
 */
  inline bool fileExists(const std::string & filepath) override;

private:
  /**
   * @brief Get all nodes from the features
   * @param features The features to grab the graph nodes from
   * @param[out] nodes The nodes found within the features
   */
  void getNodes(const Json & features, std::vector<Json> & nodes);

  /**
   * @brief Get all edges from the features
   * @param features The features to grab the graph edges from
   * @param edges The edges found with the features
   */
  void getEdges(const Json & features, std::vector<Json> & edges);

  /**
   * @brief Add all nodes into the graph
   * @param graph The graph in which the nodes are added into
   * @param graph_to_id_map A map of node id to the graph id
   * @param nodes The nodes to be added into the graph
   */
  void addNodesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes);

  /**
   * @brief Add all edges into the graph
   * @param graph The graph in which the edges are added into
   * @param edges The edges to be added into the graph
   */
  void addEdgesToGraph(nav2_route::Graph & graph, std::vector<Json> & edges);
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__GEOJSON_GRAPH_FILE_LOADER_HPP_
