// Copyright (c) 2026 Panav Arpit Raaj
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

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_route/interfaces/graph_file_loader.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/service_client.hpp"
#include "robot_localization/srv/from_ll_array.hpp"

#ifndef NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__OSM_GRAPH_FILE_LOADER_HPP_
#define NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__OSM_GRAPH_FILE_LOADER_HPP_

namespace nav2_route
{

/**
 * @class nav2_route::OsmGraphFileLoader
 * @brief A GraphFileLoader plugin to load an OpenStreetMap .osm graph representation
 */
class OsmGraphFileLoader : public GraphFileLoader
{
public:
  /**
   * @brief Constructor
   */
  OsmGraphFileLoader() = default;

  /**
   * @brief Destructor
   */
  ~OsmGraphFileLoader() = default;

  /**
   * @brief Configure, but do not store the node
   * @param node pointer to the user's lifecycle node
   * @param name name of the plugin
   */
  void configure(
    const nav2::LifecycleNode::SharedPtr node,
    const std::string & name) override;

  /**
   * @brief Loads the OSM file into the graph
   * @param graph The graph to be populated by the OSM file
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
   * @brief A way from the OSM file: an ordered list of node ids, plus its tags
   */
  struct OsmWay
  {
    std::vector<int64_t> refs;
    std::unordered_map<std::string, std::string> tags;
  };

  /**
   * @brief The part of a way between two junctions, held in node_chain
   */
  struct Section
  {
    std::vector<int64_t> node_chain;
    std::unordered_map<std::string, std::string> tags;
  };

  /**
   * @brief Checks if a file exists on the filesystem
   * @param filepath The filepath to be checked
   * @return True if the file path provided exists
   */
  bool doesFileExist(const std::string & filepath);

  /**
   * @brief Parse the OSM XML file into in-memory tables
   * @param[in] filepath The path of the .osm file to parse
   * @param[out] osm_nodes Map of OSM node id to its (latitude, longitude)
   * @param[out] kept_ways The parsed ways
   * @return True if the file was parsed successfully
   */
  bool parseOsm(
    const std::string & filepath,
    std::unordered_map<int64_t, std::pair<double, double>> & osm_nodes,
    std::vector<OsmWay> & kept_ways);

  /**
   * @brief Count how many times each node id is used across all ways
   * @param ways The parsed ways
   * @return A map of node id to how many times it is used
   */
  std::unordered_map<int64_t, size_t> countNodeReferences(const std::vector<OsmWay> & ways);

  /**
   * @brief Split each way at its junctions, the nodes used more than once
   * @param ways The parsed ways
   * @param ref_count Use counts from countNodeReferences
   * @return The sections, each running from one junction to the next
   */
  std::vector<Section> splitWaysIntoSections(
    const std::vector<OsmWay> & ways,
    const std::unordered_map<int64_t, size_t> & ref_count);

  /**
   * @brief Collect the junction ids at the ends of the sections, sorted
   * @param sections The sections between junctions
   * @return The sorted, deduplicated node ids that become graph nodes
   */
  std::vector<int64_t> collectVertexIds(const std::vector<Section> & sections);

  /**
   * @brief Convert lat/lon to map frame x/y using the FromLLArray service
   * @param[in] osm_nodes Map of OSM node id to its (latitude, longitude)
   * @param[in] ids The node ids to convert
   * @param[out] coords_out Map of node id to its map frame coordinates
   * @return True if the service converted the requested points
   */
  bool convertCoordinates(
    const std::unordered_map<int64_t, std::pair<double, double>> & osm_nodes,
    const std::vector<int64_t> & ids,
    std::unordered_map<int64_t, Coordinates> & coords_out);

  /**
   * @brief Add a graph node for each junction that has coordinates
   * @param[out] graph The graph to populate with nodes
   * @param[out] graph_to_id_map Map of OSM node id to graph index
   * @param[in] vertex_ids The junction ids that should become nodes
   * @param[in] coords Map of node id to map frame coordinates
   */
  void addNodesToGraph(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    const std::vector<int64_t> & vertex_ids,
    const std::unordered_map<int64_t, Coordinates> & coords);

  /**
   * @brief Which way traffic may travel along a way
   */
  enum class OneWay {FORWARD, REVERSE, BOTH};

  /**
   * @brief Read a way's oneway tag; anything unrecognized is treated as BOTH
   * @param tags The way's key-value tags
   * @return The direction traffic may travel
   */
  OneWay parseOneway(const std::unordered_map<std::string, std::string> & tags);

  /**
   * @brief Add one edge per section, or two if the way is not oneway
   * @param[out] graph The graph whose nodes gain outgoing edges
   * @param[in] graph_to_id_map Map of OSM node id to graph index
   * @param[in] sections The sections between junctions
   */
  void addEdgesFromSections(
    Graph & graph, GraphToIDMap & graph_to_id_map, const std::vector<Section> & sections);

  rclcpp::Logger logger_{rclcpp::get_logger("OsmGraphFileLoader")};

  nav2::ServiceClient<robot_localization::srv::FromLLArray>::SharedPtr from_ll_client_;
  std::string from_ll_service_name_{"fromLLArray"};
  double from_ll_service_timeout_{5.0};

  // Monotonic counter for synthesised edge ids (OSM has no per-segment id).
  uint64_t next_edge_id_{0};
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__OSM_GRAPH_FILE_LOADER_HPP_
