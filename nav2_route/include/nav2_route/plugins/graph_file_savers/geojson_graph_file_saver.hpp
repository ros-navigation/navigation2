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

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/interfaces/graph_file_saver.hpp"
#include "nav2_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "nav2_util/lifecycle_node.hpp"

#ifndef NAV2_ROUTE__PLUGINS__GRAPH_FILE_SAVERS__GEOJSON_GRAPH_FILE_SAVER_HPP_
#define NAV2_ROUTE__PLUGINS__GRAPH_FILE_SAVERS__GEOJSON_GRAPH_FILE_SAVER_HPP_

namespace nav2_route
{

/**
 * @class nav2_route::GeoJsonGraphFileSaver
 * @brief A GraphFileSaver plugin to save a geojson graph representation
 */
class GeoJsonGraphFileSaver : public GraphFileSaver
{
public:
  using Json = nlohmann::json;

  /**
   * @brief Constructor
   */
  GeoJsonGraphFileSaver() = default;

  /**
   * @brief Destructor
   */
  ~GeoJsonGraphFileSaver() = default;

  /**
   * @brief Configure, but do not store the node
   * @param parent pointer to user's node
   */
  void configure(
    const nav2_util::LifecycleNode::SharedPtr node) override;

  /**
   * @brief Saves the graph to a geojson file
   * @param graph The graph to save to the geojson file
   * @param filepath The path to save the graph to
   * @return True if successful
   */
  bool saveGraphToFile(
    Graph & graph,
    std::string filepath) override;

protected:
  /**
   * @brief Add nodes into the graph
   * @param[out] graph The graph that will contain the new nodes
   * @param[in] json_features Json array to add the nodes to
   */
  void loadNodesFromGraph(Graph & graph, std::vector<Json> & json_features);

  /**
   * @brief Add edges into the graph
   * @param[out] graph The graph that will contain the new edges
   * @param[in] json_edges Json array to add the edges to
   */
  void loadEdgesFromGraph(Graph & graph, std::vector<Json> & json_edges);

  /**
   * @brief Convert graph metadata to Json
   * @param metadata Metadata from a node or edge in the graph
   * @param json_metadata Json entry to store metadata in
   */
  void convertMetaDataToJson(const Metadata & metadata, Json & json_metadata);

  /**
   * @brief Convert graph operation to Json
   * @param Operations Operations information from the graph
   * @param json_operations Json entry to store operation data in
   */
  void convertOperationsToJson(const Operations & operations, Json & json_operations);

  rclcpp::Logger logger_{rclcpp::get_logger("GeoJsonGraphFileSaver")};
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__GRAPH_FILE_SAVERS__GEOJSON_GRAPH_FILE_SAVER_HPP_
