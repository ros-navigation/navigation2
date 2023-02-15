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
    const auto properties = node["properties"];

    if (properties.contains("frame")) {
      graph[idx].coords.frame_id = properties["frame"];
    }

    graph[idx].nodeid = properties["id"];

    Json geometry = node["geometry"];
    graph[idx].coords.x = geometry["coordinates"][0];
    graph[idx].coords.y = geometry["coordinates"][1];
    graph_to_id_map[graph[idx].nodeid] = idx;

    // Operations
    if (properties.contains("operations")) {
      Operations operations;
      for (const auto & operation : properties["operations"]) {
        Operation operate;
        fromJsonToOperation(operation, operate);
        operations.push_back(operate);
      }
      graph[idx].operations = operations;
    }

    // MetaData
    Metadata metadata = getMetaData(properties);
    graph[idx].metadata = metadata;
    idx++;
  }
}

void GeoJsonGraphFileLoader::addEdgesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges)
{
  for (const auto & edge : edges) {
    // Required data
    const auto properties = edge["properties"];
    unsigned int id = properties["id"];
    unsigned int start_id = properties["startid"];
    unsigned int end_id = properties["endid"];

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
    if (properties.contains("cost")) {
      edge_cost.cost = properties["cost"];
    }

    if (properties.contains("overridable")) {
      edge_cost.overridable = properties["overridable"];
    }

    // Metadata
    Metadata metadata = getMetaData(properties);

    graph[graph_to_id_map[start_id]].addEdge(
      edge_cost, &graph[graph_to_id_map[end_id]], id,
      metadata);
  }
}

Metadata GeoJsonGraphFileLoader::getMetaData(const Json & properties)
{
  Metadata metadata;
  if (properties.contains("metadata")) {
    for (const auto & data : properties["metadata"].items()) {
      if (!data.value().is_primitive()) {continue;}

      if (data.value().is_number()) {
        if (data.value().is_number_integer()) {
          int value = data.value();
          metadata.setValue(data.key(), value);
          continue;
        } else if (data.value().is_number_float()) {
          float value = data.value();
          metadata.setValue(data.key(), value);
          continue;
        } else {
          double value = data.value();
          metadata.setValue(data.key(), value);
          continue;
        }
      }

      if (data.value().is_boolean()) {
        bool value = data.value();
        metadata.setValue(data.key(), value);
        continue;
      }

      if (data.value().is_string()) {
        std::string value = data.value();
        metadata.setValue(data.key(), value);
        continue;
      }

      RCLCPP_ERROR(
        logger_, "Failed to convert the key: %s value", data.key().c_str());
      throw std::runtime_error("Failed to convert");
    }
  }
  return metadata;
}
void GeoJsonGraphFileLoader::fromJsonToOperation(const Json &json,
                                                 Operation &operation)
{
  json.at("type").get_to(operation.type);
  Json trigger = json.at("trigger");
  operation.trigger = trigger.get<OperationTrigger>();
  Metadata metadata = getMetaData(json);
  operation.metadata = metadata;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::GeoJsonGraphFileLoader, nav2_route::GraphFileLoader)
