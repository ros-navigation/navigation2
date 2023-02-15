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

#include "nav2_core/route_exceptions.hpp"
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
   * @brief Configure the scorer, but do not store the node
   * @param parent pointer to user's node
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

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
  bool fileExists(const std::string & filepath) override;

protected:
  /**
   * @brief Get the graph elements from the features
   * @param[in] features The features to grab the graph nodes from
   * @param[out] nodes The nodes found within the features
   * @param[out] edges The edges found within the features
   */
  void getGraphElements(
    const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges);

  /**
   * @brief Add all nodes into the graph
   * @param[out] graph The graph in which the nodes are added into
   * @param[out] graph_to_id_map A map of node id to the graph id
   * @param[in] nodes The nodes to be added into the graph
   */
  void addNodesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes);

  /**
   * @brief Add all edges into the graph
   * @param[out] graph The graph in which the edges are added into
   * @param[in] graph_to_id_map A map of node id to the graph id
   * @param[in] edges The edges to be added into the graph
   */
  void addEdgesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges);

  /**
   * @brief Populates the coordinate data
   * @param node The json object that holds the coordinate data
   * @return Coordinates The converted coordinate data
   */
  Coordinates getCoordinates(const Json & node);

  /**
   * @brief Populated the mete data if present in the properties tag
   * @param properties The json object that holds the metadata
   * @return Metadata The converted metadata
   */
  Metadata getMetaData(const Json & properties);

  /**
   * @brief Populates the operation data
   * @param json_operation The json object that holds the operation data
   * @return Operation The converted operation data
   */
  Operation getOperation(const Json & json_operation);

  /**
   * @brief Populates the operations if present in the properties tag
   * @param properties The json object that holds the operations data
   * @return Operations The converted operations data
   */
  Operations getOperations(const Json & properties);

  /**
   * @brief Populates the edge cost if present in the properties tag
   * @param properties The json object that holds the edge cost data
   * @return EdgeCost The converted edge cost data
   */
  EdgeCost getEdgeCost(const Json & properties);

  rclcpp::Logger logger_{rclcpp::get_logger("GeoJsonGraphFileLoader")};
};

NLOHMANN_JSON_SERIALIZE_ENUM(
  OperationTrigger, {
    {OperationTrigger::NODE, "NODE"},
    {OperationTrigger::ON_ENTER, "ON_ENTER"},
    {OperationTrigger::ON_EXIT, "ON_EXIT"},
  })

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__GEOJSON_GRAPH_FILE_LOADER_HPP_
