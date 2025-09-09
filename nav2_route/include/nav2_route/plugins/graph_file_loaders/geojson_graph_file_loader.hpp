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

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/interfaces/graph_file_loader.hpp"
#include "nav2_util/lifecycle_node.hpp"

#ifndef NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__GEOJSON_GRAPH_FILE_LOADER_HPP_
#define NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__GEOJSON_GRAPH_FILE_LOADER_HPP_

namespace nav2_route
{

/**
 * @class nav2_route::GeoJsonGraphFileLoader
 * @brief A GraphFileLoader plugin to load a geojson graph representation
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
   * @brief Configure, but do not store the node
   * @param parent pointer to user's node
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr node) override;

  /**
   * @brief Loads the geojson file into the graph
   * @param graph The graph to be populated by the geojson file
   * @param graph_to_id_map A map of node id's to the graph index
   * @param filepath The path of the file to load
   * @return True if the graph was successfully loaded
   */
  bool loadGraphFromFile(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    std::string filepath) override;

protected:
  /**
   * @brief Checks if a file even exists on the filesystem
   * @param filepath The filepath to be checked
   * @return True if the file path provided exists
   */
  bool doesFileExist(const std::string & filepath);

  /**
   * @brief Get the nodes and edges from features
   * @param[in] features The features that contain the nodes and edges
   * @param[out] nodes The nodes found within the features
   * @param[out] edges The edges found within the features
   */
  void getGraphElements(
    const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges);

  /**
   * @brief Add nodes into the graph
   * @param[out] graph The graph that will contain the new nodes
   * @param[out] graph_to_id_map A map of node id to the graph index
   * @param[in] nodes The nodes to be added into the graph
   */
  void addNodesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes);

  /**
   * @brief Add edges into the graph
   * @param[out] graph The graph that will contain the new edges
   * @param[in] graph_to_id_map A map of node id to the graph id
   * @param[in] edges The edges to be added into the graph
   */
  void addEdgesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges);

  /**
   * @brief Converts the coordinates from the json object into the Coordinates type
   * @param node The json object that holds the coordinate data
   * @return The coordinates found in the json object
   */
  Coordinates convertCoordinatesFromJson(const Json & node);

  /**
   * @brief Converts the metadata from the json object into the metadata type
   * @param properties The json object that holds the metadata
   * @param key The key for the metadata
   * @return The converted metadata
   */
  Metadata convertMetaDataFromJson(const Json & properties, const std::string & key = "metadata");

  /**
   * @brief Converts the operation from the json object into the operation type
   * @param json_operation The json object that holds the operation data
   * @return The converted operation data
   */
  Operation convertOperationFromJson(const Json & json_operation);

  /**
   * @brief Converts the operations data from the json object into the operations type if present
   * @param properties The json object that holds the operations data
   * @return Operations The converted operations data
   */
  Operations convertOperationsFromJson(const Json & properties);

  /**
   * @brief Converts the edge cost data from the json object into the edge cost type
   * @param properties The json object that holds the edge cost data
   * @return EdgeCost The converted edge cost data
   */
  EdgeCost convertEdgeCostFromJson(const Json & properties);

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
