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

#include <chrono>
#include <cinttypes>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "nav2_route/plugins/graph_file_loaders/osm_graph_file_loader.hpp"

namespace nav2_route
{

void OsmGraphFileLoader::configure(
  const nav2::LifecycleNode::SharedPtr node,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring OSM graph file loader");
  logger_ = node->get_logger();

  const std::string prefix = name + ".";

  from_ll_service_name_ = node->declare_or_get_parameter(
    prefix + "from_ll_service", std::string("fromLLArray"));
  from_ll_service_timeout_ = node->declare_or_get_parameter(
    prefix + "from_ll_service_timeout", 5.0);

  // Use navsat_transform so the graph shares the robot's GPS origin.
  from_ll_client_ = node->create_client<robot_localization::srv::FromLLArray>(
    from_ll_service_name_, true);
}

bool OsmGraphFileLoader::loadGraphFromFile(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath)
{
  if (!doesFileExist(filepath)) {
    RCLCPP_ERROR(logger_, "The filepath %s does not exist", filepath.c_str());
    return false;
  }

  // Read the file into a node id -> lat/lon table plus the list of ways.
  std::unordered_map<int64_t, std::pair<double, double>> osm_nodes;
  std::vector<OsmWay> kept_ways;
  if (!parseOsm(filepath, osm_nodes, kept_ways)) {
    return false;
  }

  if (osm_nodes.empty() || kept_ways.empty()) {
    RCLCPP_ERROR(
      logger_,
      "The OSM graph is malformed: it contains no nodes or no ways. "
      "Please check %s", filepath.c_str());
    return false;
  }

  next_edge_id_ = 0;

  // Ways only connect where they share a node, so split them at those nodes.
  const auto ref_count = countNodeReferences(kept_ways);
  const auto sections = splitWaysIntoSections(kept_ways, ref_count);

  // Convert the junctions to map frame, add them as nodes, then link them up.
  const auto vertex_ids = collectVertexIds(sections);
  std::unordered_map<int64_t, Coordinates> coords;
  if (!convertCoordinates(osm_nodes, vertex_ids, coords)) {
    return false;
  }
  addNodesToGraph(graph, graph_to_id_map, vertex_ids, coords);
  addEdgesFromSections(graph, graph_to_id_map, sections);

  if (graph.empty()) {
    RCLCPP_ERROR(logger_, "OSM graph has no usable vertices after loading %s", filepath.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger_, "Loaded OSM graph: %zu vertices from %zu sections (%s)",
    graph.size(), sections.size(), filepath.c_str());
  return true;
}

bool OsmGraphFileLoader::doesFileExist(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
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

  // OSM ids don't fit in 32 bits, so read them as signed 64-bit. Query* fails
  // on a bad attribute instead of quietly returning 0.
  for (const tinyxml2::XMLElement * node = osm->FirstChildElement("node");
    node != nullptr; node = node->NextSiblingElement("node"))
  {
    int64_t id = 0;
    double lat = 0.0;
    double lon = 0.0;
    if (node->QueryInt64Attribute("id", &id) != tinyxml2::XML_SUCCESS ||
      node->QueryDoubleAttribute("lat", &lat) != tinyxml2::XML_SUCCESS ||
      node->QueryDoubleAttribute("lon", &lon) != tinyxml2::XML_SUCCESS)
    {
      RCLCPP_WARN(logger_, "Skipping an OSM <node> with a missing or invalid id/lat/lon");
      continue;
    }
    if (id < 0) {
      // Editors give objects that were never uploaded a temporary negative id,
      // which would wrap into our reserved ids.
      RCLCPP_WARN(
        logger_,
        "Skipping OSM node with negative id %" PRId64 "; ids must be non-negative", id);
      continue;
    }
    osm_nodes[id] = std::make_pair(lat, lon);
  }

  for (const tinyxml2::XMLElement * way = osm->FirstChildElement("way");
    way != nullptr; way = way->NextSiblingElement("way"))
  {
    OsmWay osm_way;

    for (const tinyxml2::XMLElement * nd = way->FirstChildElement("nd");
      nd != nullptr; nd = nd->NextSiblingElement("nd"))
    {
      int64_t ref = 0;
      if (nd->QueryInt64Attribute("ref", &ref) != tinyxml2::XML_SUCCESS) {
        RCLCPP_WARN(logger_, "Skipping a <nd> with a missing or invalid ref in a way");
        continue;
      }
      osm_way.refs.push_back(ref);
    }

    for (const tinyxml2::XMLElement * tag = way->FirstChildElement("tag");
      tag != nullptr; tag = tag->NextSiblingElement("tag"))
    {
      const char * key = tag->Attribute("k");
      const char * value = tag->Attribute("v");
      // Attribute() gives nullptr when missing, so check before using it.
      if (key != nullptr && value != nullptr) {
        osm_way.tags[key] = value;
      }
    }

    kept_ways.push_back(osm_way);
  }

  return true;
}

std::unordered_map<int64_t, size_t> OsmGraphFileLoader::countNodeReferences(
  const std::vector<OsmWay> & ways)
{
  std::unordered_map<int64_t, size_t> ref_count;
  for (const auto & way : ways) {
    for (const int64_t node_id : way.refs) {
      ref_count[node_id]++;
    }
  }
  return ref_count;
}

std::vector<OsmGraphFileLoader::Section> OsmGraphFileLoader::splitWaysIntoSections(
  const std::vector<OsmWay> & ways,
  const std::unordered_map<int64_t, size_t> & ref_count)
{
  std::vector<Section> sections;
  for (const auto & way : ways) {
    Section current_section;
    current_section.tags = way.tags;

    for (size_t i = 0; i < way.refs.size(); ++i) {
      const int64_t node_id = way.refs[i];

      // Skip repeated nodes, they would make zero-length edges
      if (!current_section.node_chain.empty() &&
        current_section.node_chain.back() == node_id)
      {
        continue;
      }

      current_section.node_chain.push_back(node_id);

      // A junction mid-way ends this section and starts the next one, and is
      // added to both so the two sections stay connected.
      const bool is_junction = ref_count.at(node_id) > 1;
      const bool is_interior = i + 1 < way.refs.size();
      if (is_junction && is_interior && current_section.node_chain.size() > 1) {
        sections.push_back(current_section);
        current_section = Section();
        current_section.tags = way.tags;
        current_section.node_chain.push_back(node_id);
      }
    }

    // Save the last section; a single node has no length
    if (current_section.node_chain.size() > 1) {
      sections.push_back(current_section);
    }
  }
  return sections;
}

std::vector<int64_t> OsmGraphFileLoader::collectVertexIds(
  const std::vector<Section> & sections)
{
  std::set<int64_t> unique;  // sorted and deduped, so indices come out the same
  for (const auto & section : sections) {
    if (section.node_chain.empty()) {
      continue;
    }
    unique.insert(section.node_chain.front());
    unique.insert(section.node_chain.back());
  }
  return std::vector<int64_t>(unique.begin(), unique.end());
}

bool OsmGraphFileLoader::convertCoordinates(
  const std::unordered_map<int64_t, std::pair<double, double>> & osm_nodes,
  const std::vector<int64_t> & ids,
  std::unordered_map<int64_t, Coordinates> & coords_out)
{
  if (!from_ll_client_) {
    RCLCPP_ERROR(logger_, "fromLLArray client is not configured; was configure() called?");
    return false;
  }

  // Convert all points in one call, keeping the ids in order to match the
  // reply. Nodes missing from the file are skipped and get no coordinates.
  auto request = std::make_shared<robot_localization::srv::FromLLArray::Request>();
  std::vector<int64_t> request_ids;
  request_ids.reserve(ids.size());
  for (const int64_t id : ids) {
    const auto it = osm_nodes.find(id);
    if (it == osm_nodes.end()) {
      RCLCPP_WARN(
        logger_, "OSM node %" PRId64 " referenced by a way is missing; skipping it", id);
      continue;
    }
    geographic_msgs::msg::GeoPoint point;
    point.latitude = it->second.first;
    point.longitude = it->second.second;
    point.altitude = 0.0;
    request->ll_points.push_back(point);
    request_ids.push_back(id);
  }

  if (request->ll_points.empty()) {
    RCLCPP_ERROR(logger_, "No OSM nodes had usable coordinates to convert");
    return false;
  }

  auto response = std::make_shared<robot_localization::srv::FromLLArray::Response>();
  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(from_ll_service_timeout_));
  if (!from_ll_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      logger_,
      "FromLLArray service '%s' unavailable after %.1fs - is navsat_transform_node running?",
      from_ll_service_name_.c_str(), from_ll_service_timeout_);
    return false;
  }
  if (!from_ll_client_->invoke(request, response)) {
    RCLCPP_ERROR(
      logger_, "FromLLArray service '%s' call failed", from_ll_service_name_.c_str());
    return false;
  }

  if (response->map_points.size() != request_ids.size()) {
    RCLCPP_ERROR(
      logger_, "fromLLArray returned %zu points for %zu requested",
      response->map_points.size(), request_ids.size());
    return false;
  }

  for (size_t i = 0; i < request_ids.size(); ++i) {
    Coordinates coords;
    coords.frame_id = "map";  // navsat_transform returns map frame coordinates
    coords.x = static_cast<float>(response->map_points[i].x);
    coords.y = static_cast<float>(response->map_points[i].y);
    coords_out[request_ids[i]] = coords;
  }
  return true;
}

void OsmGraphFileLoader::addNodesToGraph(
  Graph & graph,
  GraphToIDMap & graph_to_id_map,
  const std::vector<int64_t> & vertex_ids,
  const std::unordered_map<int64_t, Coordinates> & coords)
{
  // Only junctions with coordinates can become graph nodes.
  std::vector<int64_t> usable;
  usable.reserve(vertex_ids.size());
  for (const int64_t id : vertex_ids) {
    if (coords.count(id) > 0) {
      usable.push_back(id);
    }
  }

  graph.resize(usable.size());
  for (size_t idx = 0; idx < usable.size(); ++idx) {
    const int64_t osm_id = usable[idx];
    // Node ids are 64-bit, so keep the OSM id as-is (parseOsm already
    // rejected negatives, which would land on our reserved ids).
    const auto nodeid = static_cast<uint64_t>(osm_id);
    graph[idx].nodeid = nodeid;
    // Allows callers to look up a graph index by OSM id.
    graph_to_id_map[nodeid] = static_cast<uint64_t>(idx);
    graph[idx].coords = coords.at(osm_id);
  }
}

OsmGraphFileLoader::OneWay OsmGraphFileLoader::parseOneway(
  const std::unordered_map<std::string, std::string> & tags)
{
  const auto it = tags.find("oneway");
  if (it == tags.end()) {
    return OneWay::BOTH;
  }

  const std::string & value = it->second;
  if (value == "yes" || value == "true" || value == "1") {
    return OneWay::FORWARD;
  }
  if (value == "-1" || value == "reverse") {
    return OneWay::REVERSE;
  }
  if (value == "no" || value == "false" || value == "0") {
    return OneWay::BOTH;
  }

  RCLCPP_WARN(
    logger_, "Unrecognized oneway value '%s'; treating the way as bidirectional",
    value.c_str());
  return OneWay::BOTH;
}

void OsmGraphFileLoader::addEdgesFromSections(
  Graph & graph, GraphToIDMap & graph_to_id_map, const std::vector<Section> & sections)
{
  for (const auto & section : sections) {
    if (section.node_chain.size() < 2) {
      continue;
    }

    const auto start_it = graph_to_id_map.find(static_cast<uint64_t>(section.node_chain.front()));
    const auto end_it = graph_to_id_map.find(static_cast<uint64_t>(section.node_chain.back()));
    if (start_it == graph_to_id_map.end() || end_it == graph_to_id_map.end()) {
      // One end never became a node, so there is nothing to connect.
      RCLCPP_WARN(logger_, "Skipping a section with an unresolved boundary node");
      continue;
    }

    const uint64_t start_index = start_it->second;
    const uint64_t end_index = end_it->second;
    if (start_index == end_index) {
      // The section loops back to where it started. An edge to itself is no
      // use for routing, so drop it.
      RCLCPP_WARN(
        logger_,
        "Dropping self-loop section at junction %" PRIu64 " (closed spur with no second junction)",
        graph[start_index].nodeid);
      continue;
    }

    // Leave the cost at its default; the edge scorers work it out from the
    // node positions, same as a GeoJSON edge without a cost.
    EdgeCost cost;
    const OneWay direction = parseOneway(section.tags);
    if (direction == OneWay::FORWARD || direction == OneWay::BOTH) {
      graph[start_index].addEdge(cost, &graph[end_index], next_edge_id_++);
    }
    if (direction == OneWay::REVERSE || direction == OneWay::BOTH) {
      graph[end_index].addEdge(cost, &graph[start_index], next_edge_id_++);
    }
  }
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::OsmGraphFileLoader, nav2_route::GraphFileLoader)
