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

#include <tinyxml2.h>

#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_route/plugins/graph_file_loaders/osm_graph_file_loader.hpp"

namespace nav2_route
{

void OsmGraphFileLoader::configure(const nav2::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(node->get_logger(), "Configuring OSM graph file loader");
  logger_ = node->get_logger();

  const std::string prefix = "osm_graph_file_loader.";
  std::vector<std::string> highways =
    node->declare_or_get_parameter(prefix + "highway_filter", std::vector<std::string>{});
  highway_filter_ = std::unordered_set<std::string>(highways.begin(), highways.end());

  use_datum_override_ = node->declare_or_get_parameter(prefix + "use_datum_override", false);
  datum_lat_ = node->declare_or_get_parameter(prefix + "datum_latitude", 0.0);
  datum_lon_ = node->declare_or_get_parameter(prefix + "datum_longitude", 0.0);
  bearing_threshold_deg_ =
    node->declare_or_get_parameter(prefix + "bearing_threshold_deg", 0.0);
}

bool OsmGraphFileLoader::loadGraphFromFile(
  Graph & /*graph*/, GraphToIDMap & /*graph_to_id_map*/, std::string filepath)
{
  if (!doesFileExist(filepath)) {
    RCLCPP_ERROR(logger_, "The filepath %s does not exist", filepath.c_str());
    return false;
  }

  // Stage 1 (this milestone): turn the XML text into two in-memory tables and
  // stop. Interpreting the topology happens in later milestones.
  std::unordered_map<int64_t, std::pair<double, double>> osm_nodes;
  std::vector<OsmWay> kept_ways;
  if (!parseOsm(filepath, osm_nodes, kept_ways)) {
    return false;
  }

  if (osm_nodes.empty() || kept_ways.empty()) {
    RCLCPP_ERROR(
      logger_,
      "The OSM graph is malformed: it contains no nodes or no usable highway ways. "
      "Please check %s", filepath.c_str());
    return false;
  }

  // M2 stops at parsing. Topology resolution (M3), coordinate conversion (M4)
  // and edge construction (M5) still populate the graph, so report failure for
  // now to signal the graph is not yet built.
  RCLCPP_INFO(
    logger_, "Parsed %zu OSM nodes and %zu kept ways from %s",
    osm_nodes.size(), kept_ways.size(), filepath.c_str());
  return false;
}

bool OsmGraphFileLoader::doesFileExist(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}

bool OsmGraphFileLoader::shouldKeepWay(
  const std::unordered_map<std::string, std::string> & tags)
{
  auto it = tags.find("highway");
  if (it == tags.end()) {
    // Not a highway way (e.g. a building outline or area) - drop it.
    return false;
  }

  if (highway_filter_.empty()) {
    // No allowlist configured: keep every highway=* way.
    return true;
  }

  return highway_filter_.count(it->second) > 0;
}

bool OsmGraphFileLoader::parseOsm(
  const std::string & filepath,
  std::unordered_map<int64_t, std::pair<double, double>> & osm_nodes,
  std::vector<OsmWay> & kept_ways)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger_, "Failed to parse OSM XML file %s", filepath.c_str());
    return false;
  }

  const tinyxml2::XMLElement * osm = doc.RootElement();
  if (osm == nullptr) {
    RCLCPP_ERROR(logger_, "OSM file %s has no root element", filepath.c_str());
    return false;
  }

  // Every <node> becomes an entry in the id -> (lat, lon) table. OSM ids are
  // 64-bit, so we always read them with Int64Attribute - never an unsigned
  // accessor, which would silently truncate.
  for (const tinyxml2::XMLElement * node = osm->FirstChildElement("node");
    node != nullptr; node = node->NextSiblingElement("node"))
  {
    const int64_t id = node->Int64Attribute("id");
    const double lat = node->DoubleAttribute("lat");
    const double lon = node->DoubleAttribute("lon");
    osm_nodes[id] = std::make_pair(lat, lon);
  }

  for (const tinyxml2::XMLElement * way = osm->FirstChildElement("way");
    way != nullptr; way = way->NextSiblingElement("way"))
  {
    OsmWay osm_way;

    for (const tinyxml2::XMLElement * nd = way->FirstChildElement("nd");
      nd != nullptr; nd = nd->NextSiblingElement("nd"))
    {
      osm_way.refs.push_back(nd->Int64Attribute("ref"));
    }

    for (const tinyxml2::XMLElement * tag = way->FirstChildElement("tag");
      tag != nullptr; tag = tag->NextSiblingElement("tag"))
    {
      const char * key = tag->Attribute("k");
      const char * value = tag->Attribute("v");
      // Attribute() returns nullptr if the attribute is absent; guard before
      // constructing a std::string from it.
      if (key != nullptr && value != nullptr) {
        osm_way.tags[key] = value;
      }
    }

    if (shouldKeepWay(osm_way.tags)) {
      kept_ways.push_back(osm_way);
    }
  }

  return true;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::OsmGraphFileLoader, nav2_route::GraphFileLoader)
