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

#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "nav2_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"

namespace nav2_route
{

bool GeoJsonGraphFileLoader::loadGraphFromFile(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath)
{
  std::ifstream graph_file(filepath);

  Json json;
  graph_file >> json;

  auto features = json.at("features");
  std::vector<Json> nodes;
  std::vector<Json> edges;
  getNodes(features, nodes);
  getEdges(features, edges);

  graph.resize(nodes.size());
  addNodesToGraph(graph, graph_to_id_map, nodes);
  addEdgesToGraph(graph, edges);

  return true;
}

bool GeoJsonGraphFileLoader::fileExists(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}


void GeoJsonGraphFileLoader::getNodes(const Json & features, std::vector<Json> & nodes)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    }
  }
}

void GeoJsonGraphFileLoader::getEdges(const Json & features, std::vector<Json> & edges)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "MultiLineString") {
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
    unsigned int id = node["properties"]["id"];
    float x = node["geometry"]["coordinates"][0];
    float y = node["geometry"]["coordinates"][1];
    graph[id].nodeid = id;
    graph[id].coords.x = x;
    graph[id].coords.y = y;
    graph_to_id_map[graph[idx].nodeid] = idx;
    idx++;
  }
}

void GeoJsonGraphFileLoader::addEdgesToGraph(Graph & graph, std::vector<Json> & edges)
{
  nav2_route::EdgeCost edge_cost;
  for (const auto & edge : edges) {
    // Required data
    const auto edge_properties = edge["properties"];
    unsigned int id = edge_properties["id"];
    unsigned int start_id = edge_properties["startid"];
    unsigned int end_id = edge_properties["endid"];

    // Recommended data
    if (edge_properties.contains("cost")) {
      edge_cost.cost = edge_properties["cost"];
    }

    if (edge_properties.contains("overridable")) {
      edge_cost.overridable = edge_properties["overridable"];
    }

    graph[start_id].addEdge(edge_cost, &graph[end_id], id);
  }
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::GeoJsonGraphFileLoader, nav2_route::GraphFileLoader)
