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

void OsmGraphFileLoader::configure(const nav2::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(node->get_logger(), "Configuring OSM graph file loader");
  logger_ = node->get_logger();

  const std::string prefix = "osm_graph_file_loader.";
  std::vector<std::string> highways =
    node->declare_or_get_parameter(prefix + "highway_filter", std::vector<std::string>{});
  highway_filter_ = std::unordered_set<std::string>(highways.begin(), highways.end());

  from_ll_service_name_ = node->declare_or_get_parameter(
    prefix + "from_ll_service", std::string("/fromLLArray"));
  from_ll_service_timeout_ = node->declare_or_get_parameter(
    prefix + "from_ll_service_timeout", 5.0);

  // Convert OSM lat/lon through robot_localization's navsat_transform, so the
  // datum (and the resulting map frame) is shared with the robot's localization
  // rather than introducing a second, independent datum here. The client owns
  // its own internal executor so it can be invoked synchronously at load time.
  from_ll_client_ = node->create_client<robot_localization::srv::FromLLArray>(
    from_ll_service_name_, true /* creates and spins an internal executor */);
}

bool OsmGraphFileLoader::loadGraphFromFile(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath)
{
  if (!doesFileExist(filepath)) {
    RCLCPP_ERROR(logger_, "The filepath %s does not exist", filepath.c_str());
    return false;
  }

  // Parse the XML into two in-memory tables: node id -> lat/lon, and the
  // highway-filtered ways.
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

  osm_to_nodeid_.clear();
  next_edge_id_ = 0;

  // Resolve the implicit topology: ways connect only where they share a node
  // id, so shared nodes (junctions) split ways into sections, and each section
  // becomes one edge between two junction vertices.
  const auto ref_count = countNodeReferences(kept_ways);
  const auto sections = splitWaysIntoSections(kept_ways, ref_count);

  // Project the junction coordinates into the robot's map frame and make them
  // graph vertices, then wire the sections between them as directed edges.
  const auto vertex_ids = collectVertexIds(sections);
  std::unordered_map<int64_t, Coordinates> coords;
  if (!convertCoordinates(osm_nodes, vertex_ids, coords)) {
    return false;
  }
  addNodesToGraph(graph, graph_to_id_map, vertex_ids, coords);
  addEdgesFromSections(graph, sections);

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
  // accessor, which would silently truncate. The Query* accessors report a
  // missing/unparseable attribute instead of silently returning 0, so a
  // malformed node is skipped rather than corrupting the table (e.g. several
  // id-less nodes colliding at id 0).
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

      // Consecutive duplicate refs would create zero-length segments
      if (!current_section.node_chain.empty() &&
        current_section.node_chain.back() == node_id)
      {
        continue;
      }

      current_section.node_chain.push_back(node_id);

      // A junction interior to the way closes the current section and opens
      // the next one. The junction id ends one chain AND begins the other:
      // sharing that boundary node is what stitches the network together.
      const bool is_junction = ref_count.at(node_id) > 1;
      const bool is_interior = i + 1 < way.refs.size();
      if (is_junction && is_interior && current_section.node_chain.size() > 1) {
        sections.push_back(current_section);
        current_section = Section();
        current_section.tags = way.tags;
        current_section.node_chain.push_back(node_id);
      }
    }

    // Flush the final run of the way; a single-node chain has no extent
    if (current_section.node_chain.size() > 1) {
      sections.push_back(current_section);
    }
  }
  return sections;
}

std::vector<int64_t> OsmGraphFileLoader::collectVertexIds(
  const std::vector<Section> & sections)
{
  std::set<int64_t> unique;  // ordered + deduped -> deterministic graph indices
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

  // Build the batch request, tracking which id each entry corresponds to so the
  // response can be mapped back. Ids missing from the file (clipped extracts)
  // are skipped here, so they simply never gain coordinates.
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
    coords.frame_id = "map";  // LocalCartesian-style output from navsat_transform
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
  // Only junctions that actually have coordinates can become vertices.
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
    const auto nav2_id = static_cast<unsigned int>(idx);
    graph[idx].nodeid = nav2_id;
    // graph_to_id_map translates an external node id to a graph index. Our
    // external ids are the sequential ids we just assigned, so this is the
    // identity map (the OSM int64 ids live separately in osm_to_nodeid_).
    graph_to_id_map[nav2_id] = nav2_id;
    osm_to_nodeid_[osm_id] = nav2_id;
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
  Graph & graph, const std::vector<Section> & sections)
{
  for (const auto & section : sections) {
    if (section.node_chain.size() < 2) {
      continue;
    }

    const auto start_it = osm_to_nodeid_.find(section.node_chain.front());
    const auto end_it = osm_to_nodeid_.find(section.node_chain.back());
    if (start_it == osm_to_nodeid_.end() || end_it == osm_to_nodeid_.end()) {
      // A boundary junction had no coordinates (e.g. clipped extract) and so
      // never became a vertex; this section cannot be connected.
      RCLCPP_WARN(logger_, "Skipping a section with an unresolved boundary node");
      continue;
    }

    const unsigned int start_id = start_it->second;
    const unsigned int end_id = end_it->second;
    if (start_id == end_id) {
      // Section that loops back to its own start (e.g. a closed spur attached
      // to the network at a single junction). A self edge is useless for
      // routing, so it is dropped - which also drops the spur's interior.
      RCLCPP_WARN(
        logger_, "Dropping self-loop section at junction %u (closed spur with no second junction)",
        start_id);
      continue;
    }

    // Default cost: the edge scorers (DistanceScorer) compute the traversal
    // cost from the vertex coordinates at query time, exactly as for a
    // GeoJSON edge with no explicit cost.
    EdgeCost cost;
    const OneWay direction = parseOneway(section.tags);
    if (direction == OneWay::FORWARD || direction == OneWay::BOTH) {
      graph[start_id].addEdge(cost, &graph[end_id], next_edge_id_++);
    }
    if (direction == OneWay::REVERSE || direction == OneWay::BOTH) {
      graph[end_id].addEdge(cost, &graph[start_id], next_edge_id_++);
    }
  }
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::OsmGraphFileLoader, nav2_route::GraphFileLoader)
