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

#include "nav2_route/interfaces/graph_file_loader.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

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

  rclcpp::Logger logger_{rclcpp::get_logger("OsmGraphFileLoader")};

  std::unordered_set<std::string> highway_filter_;
  bool use_datum_override_{false};
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double bearing_threshold_deg_{0.0};
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PLUGINS__GRAPH_FILE_LOADERS__OSM_GRAPH_FILE_LOADER_HPP_
