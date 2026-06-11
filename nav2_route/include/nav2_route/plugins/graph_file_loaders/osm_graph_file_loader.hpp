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
#include <unordered_set>
#include <utility>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
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
   */
  void configure(
    const nav2::LifecycleNode::SharedPtr node) override;

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
   * @brief A way buffered from the OSM file: an ordered list of node id
   * references plus its key/value tags. Note: a way stores no coordinates of
   * its own, only references to nodes - that ordered ref list is the polyline.
   */
  struct OsmWay
  {
    std::vector<int64_t> refs;
    std::unordered_map<std::string, std::string> tags;
  };

  /**
   * @brief A run of a way between two junctions. The full ordered id list is
   * held in node_chain [start junction, ...shape nodes..., end junction] so
   * the true polyline length can be computed later; only the two endpoints
   * become graph vertices.
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
   * @param[out] kept_ways The ways retained after highway filtering
   * @return True if the file was parsed successfully
   */
  bool parseOsm(
    const std::string & filepath,
    std::unordered_map<int64_t, std::pair<double, double>> & osm_nodes,
    std::vector<OsmWay> & kept_ways);

  /**
   * @brief Decide whether a way should be retained based on its highway tag
   * @param tags The way's key/value tags
   * @return True if the way has a highway tag allowed by the filter
   */
  bool shouldKeepWay(const std::unordered_map<std::string, std::string> & tags);

  /**
   * @brief Count how many times each node id is referenced across all kept
   * ways, including repeat visits within a single way
   * @param ways The ways retained after highway filtering
   * @return A map of node id to its reference count across kept ways
   */
  std::unordered_map<int64_t, size_t> countNodeReferences(const std::vector<OsmWay> & ways);

  /**
   * @brief Split each way at its junction nodes (nodes with reference count
   * > 1) into sections running junction to junction
   * @param ways The ways retained after highway filtering
   * @param ref_count Reference count from countNodeReferences
   * @return A vector of sections, each a run of a way between two junctions
   */
  std::vector<Section> splitWaysIntoSections(
    const std::vector<OsmWay> & ways,
    const std::unordered_map<int64_t, size_t> & ref_count);

  /**
   * @brief Collect the unique junction ids that bound the sections; these are
   * the nodes that become graph vertices (sorted for deterministic indices)
   * @param sections The inter-junction sections
   * @return Sorted unique section-boundary node ids
   */
  std::vector<int64_t> collectVertexIds(const std::vector<Section> & sections);

  /**
   * @brief Convert OSM lat/lon to map-frame x/y using robot_localization's
   * FromLLArray service, so the graph lands in the same frame the robot
   * localizes in. Ids absent from osm_nodes (e.g. clipped extracts) are
   * skipped with a warning rather than failing the whole load.
   * @param[in] osm_nodes Map of OSM node id to its (latitude, longitude)
   * @param[in] ids The node ids to convert
   * @param[out] coords_out Map of node id to its map-frame coordinates
   * @return True if the service converted at least the requested points
   */
  bool convertCoordinates(
    const std::unordered_map<int64_t, std::pair<double, double>> & osm_nodes,
    const std::vector<int64_t> & ids,
    std::unordered_map<int64_t, Coordinates> & coords_out);

  /**
   * @brief Create a graph vertex for each junction that has coordinates,
   * assigning sequential nav2 node ids and recording the OSM id mapping
   * @param[out] graph The graph to populate with nodes
   * @param[out] graph_to_id_map Map of nav2 node id to graph index
   * @param[in] vertex_ids The junction ids that should become vertices
   * @param[in] coords Map of node id to map-frame coordinates
   */
  void addNodesToGraph(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    const std::vector<int64_t> & vertex_ids,
    const std::unordered_map<int64_t, Coordinates> & coords);

  /**
   * @brief Direction of travel a way permits, from its oneway tag
   */
  enum class OneWay {FORWARD, REVERSE, BOTH};

  /**
   * @brief Interpret a way's oneway tag: yes/true/1 -> FORWARD (along the node
   * order), -1/reverse -> REVERSE, absent/no/false -> BOTH, anything else ->
   * BOTH with a warning
   * @param tags The way/section key-value tags
   * @return The permitted direction of travel
   */
  OneWay parseOneway(const std::unordered_map<std::string, std::string> & tags);

  /**
   * @brief Wire each section into the graph as one (oneway) or two (two-way)
   * DirectionalEdges between its boundary junctions. Sections whose endpoints
   * never became vertices (clipped extracts) are skipped.
   * @param[out] graph The graph whose nodes gain outgoing edges
   * @param[in] sections The inter-junction sections
   */
  void addEdgesFromSections(Graph & graph, const std::vector<Section> & sections);

  rclcpp::Logger logger_{rclcpp::get_logger("OsmGraphFileLoader")};

  nav2::ServiceClient<robot_localization::srv::FromLLArray>::SharedPtr from_ll_client_;

  std::unordered_set<std::string> highway_filter_;

  // OSM node id (int64) -> assigned nav2 node id (uint); never cast int64
  // directly into the 32-bit nav2 id, which would truncate.
  std::unordered_map<int64_t, unsigned int> osm_to_nodeid_;

  // Monotonic counter for synthesised edge ids (OSM has no per-segment id).
  unsigned int next_edge_id_{0};
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__OSM_GRAPH_FILE_LOADER_HPP_
