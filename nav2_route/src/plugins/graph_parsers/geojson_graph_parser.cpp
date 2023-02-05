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

#include "nav2_route/plugins/graph_parsers/geojson_graph_parser.hpp"

namespace nav2_route
{

bool GeoJsonGraphParser::loadGraphFromFile(Graph &graph, const std::string filepath)
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
  addNodesToGraph(graph, nodes);
  addEdgesToGraph(graph, edges);

  return true;
}

void GeoJsonGraphParser::getNodes(const Json & features, std::vector<Json> & nodes)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    }
  }
}

void GeoJsonGraphParser::getEdges(const Json & features, std::vector<Json> & edges)
{
  for (const auto & feature : features) {
    if (feature["geometry"]["type"] == "MultiLineString") {
      edges.emplace_back(feature);
    }
  }
}

void GeoJsonGraphParser::addNodesToGraph(Graph & graph, std::vector<Json> & nodes)
{
  for (const auto & node : nodes) {
    // Required data
    unsigned int id = node["properties"]["id"];
    float x = node["geometry"]["coordinates"][0];
    float y = node["geometry"]["coordinates"][1];
    graph[id].nodeid = id;
    graph[id].coords.x = x;
    graph[id].coords.y = y;
    // graph[id].coords.frame_id = frame; // TODO(jw) use default for now
  }
}

void GeoJsonGraphParser::addEdgesToGraph(Graph & graph, std::vector<Json> & edges)
{
  nav2_route::EdgeCost edge_cost;
  for (const auto & edge : edges) {
    // Required data
    const auto edge_properties = edge["properties"];
    unsigned int id = edge_properties["id"];
    unsigned int start_id = edge_properties["startid"];
    unsigned int end_id = edge_properties["endid"];

    // Recommended data
    if ( edge_properties.contains("cost"))
    {
      edge_cost.cost = edge_properties["cost"];
    }

    if ( edge_properties.contains("overridable"))
    {
      edge_cost.overridable = edge_properties["overridable"];
    }

    graph[start_id].addEdge(edge_cost, &graph[end_id], id);
  }
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::GeoJsonGraphParser, nav2_route::GraphParser)