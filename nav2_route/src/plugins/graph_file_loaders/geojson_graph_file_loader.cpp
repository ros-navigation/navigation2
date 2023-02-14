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
#include <fstream>

#include "nav2_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"

namespace nav2_route
{

void GeoJsonGraphFileLoader::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(node->get_logger(), "Configuring geojson graph file loader");
  logger_ = node->get_logger();
}

bool GeoJsonGraphFileLoader::loadGraphFromFile(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath)
{
  std::ifstream graph_file(filepath);
  Json geojson_graph;

  try {
    geojson_graph = Json::parse(graph_file);
  } catch (Json::parse_error & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse %s: %s", filepath.c_str(), ex.what());
    return false;
  }

  auto features = geojson_graph["features"];
  std::vector<Json> nodes, edges;
  getGraphElements(features, nodes, edges);

  if (nodes.empty()) {
    RCLCPP_ERROR(logger_, "No nodes were found in %s", filepath.c_str());
    return false;
  }

  if (edges.empty()) {
    RCLCPP_ERROR(logger_, "No edges were found in %s", filepath.c_str());
    return false;
  }

  graph.resize(nodes.size());
  addNodesToGraph(graph, graph_to_id_map, nodes);
  addEdgesToGraph(graph, graph_to_id_map, edges);

  return true;
}

bool GeoJsonGraphFileLoader::fileExists(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}


void GeoJsonGraphFileLoader::getGraphElements(
  const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    } else if (feature["geometry"]["type"] == "MultiLineString") {
      edges.emplace_back(feature);
    }
  }
}

void GeoJsonGraphFileLoader::addNodesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes)
{
  int idx = 0;
  for (const auto & node : nodes) {
    // Required data
    Json properties = node["properties"];

    if (properties.contains("frame")) {
      graph[idx].coords.frame_id = properties["frame"];
    }

    //TODO(jw): check for duplicate node ids
    graph[idx].nodeid = properties["id"];

    Json geometry = node["geometry"];
    graph[idx].coords.x = geometry["coordinates"][0];
    graph[idx].coords.y = geometry["coordinates"][1];
    graph_to_id_map[graph[idx].nodeid] = idx;
    idx++;
  }
}

void GeoJsonGraphFileLoader::addEdgesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges)
{
  for (const auto & edge : edges) {
    // Required data
    const auto edge_properties = edge["properties"];
    unsigned int id = edge_properties["id"];
    unsigned int start_id = edge_properties["startid"];
    unsigned int end_id = edge_properties["endid"];

    if (graph_to_id_map.find(start_id) == graph_to_id_map.end()) {
      RCLCPP_ERROR(logger_, "Start id %u does not exist for edge id %u", start_id, id);
      throw nav2_core::NoValidGraph("Start id does not exist");
    }

    if (graph_to_id_map.find(end_id) == graph_to_id_map.end()) {
      RCLCPP_ERROR(logger_, "End id of %u does not exist for edge id %u", end_id, id);
      throw nav2_core::NoValidGraph("End id does not exist");
    }

    // Recommended data

    // Edge Cost
    nav2_route::EdgeCost edge_cost;
    if (edge_properties.contains("cost")) {
      edge_cost.cost = edge_properties["cost"];
    }

    if (edge_properties.contains("overridable")) {
      edge_cost.overridable = edge_properties["overridable"];
    }

    graph[graph_to_id_map[start_id]].addEdge(edge_cost, &graph[graph_to_id_map[end_id]], id);
  }
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::GeoJsonGraphFileLoader, nav2_route::GraphFileLoader)
