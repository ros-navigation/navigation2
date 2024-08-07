// Copyright (c) 2024 Leidos
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

#include "nav2_route/plugins/graph_file_savers/geojson_graph_file_saver.hpp"

namespace nav2_route
{

void GeoJsonGraphFileSaver::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(node->get_logger(), "Configuring geojson graph file saver");
  logger_ = node->get_logger();
}

bool GeoJsonGraphFileSaver::saveGraphToFile(
  Graph & graph, std::string filepath)
{
  Json json_graph, json_crs, json_properties;
  json_graph["type"] = "FeatureCollection";
  json_graph["name"] = "graph";
  json_properties["name"] = "urn:ogc:def:crs:EPSG::3857";
  json_crs["type"] = "name";
  json_crs["properties"] = json_properties;
  json_graph["crs"] = json_crs;
  std::vector<Json> json_features;
  loadNodesFromGraph(graph, json_features);
  loadEdgesFromGraph(graph, json_features);
  json_graph["features"] = json_features;
  std::ofstream file(filepath);
  file << json_graph.dump(4) << std::endl;
  file.close();
  return true;
}

void GeoJsonGraphFileSaver::loadNodesFromGraph(
  Graph & graph, std::vector<Json> & json_features)
{
  for (const auto & node : graph) {
    Json json_feature, json_properties, json_geometry;
    json_geometry["type"] = "Point";
    json_geometry["coordinates"] = std::vector<float>{node.coords.x, node.coords.y};
    json_feature["geometry"] = json_geometry;
    json_properties["id"] = node.nodeid;
    json_feature["properties"] = json_properties;
    json_feature["type"] = "Feature";
    json_features.push_back(json_feature);
  }
}

void GeoJsonGraphFileSaver::loadEdgesFromGraph(
  Graph & graph, std::vector<Json> & json_edges)
{
  for (const auto & node : graph) {
    for (const auto & edge : node.neighbors) {
      Json json_edge, json_properties, json_geometry;
      json_geometry["type"] = "MultiLineString";
      json_edge["geometry"] = json_geometry;
      json_properties["id"] = edge.edgeid;
      json_properties["startid"] = edge.start->nodeid;
      json_properties["endid"] = edge.end->nodeid;
      json_edge["properties"] = json_properties;
      json_edge["type"] = "Feature";
      json_edges.push_back(json_edge);
    }
  }
}
}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::GeoJsonGraphFileSaver, nav2_route::GraphFileSaver)
