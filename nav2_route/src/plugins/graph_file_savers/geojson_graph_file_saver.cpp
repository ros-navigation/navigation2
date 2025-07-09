// Copyright (c) 2024 John Chrosniak
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
  const nav2_util::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(node->get_logger(), "Configuring geojson graph file saver");
  logger_ = node->get_logger();
}

bool GeoJsonGraphFileSaver::saveGraphToFile(
  Graph & graph, std::string filepath)
{
  if (filepath.empty()) {
    RCLCPP_ERROR(logger_, "File path is empty");
    return false;
  }
  Json json_graph, json_crs, json_properties;
  json_graph["type"] = "FeatureCollection";
  json_graph["name"] = "graph";
  json_properties["name"] = "urn:ogc:def:crs:EPSG::3857";
  json_crs["type"] = "name";
  json_crs["properties"] = json_properties;
  json_graph["crs"] = json_crs;
  std::vector<Json> json_features;
  try {
    loadNodesFromGraph(graph, json_features);
    loadEdgesFromGraph(graph, json_features);
    json_graph["features"] = json_features;
    std::ofstream file(filepath);
    file << json_graph.dump(4) << std::endl;
    file.close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "An error occurred: %s", e.what());
    return false;
  }
  return true;
}

void GeoJsonGraphFileSaver::loadNodesFromGraph(
  Graph & graph, std::vector<Json> & json_features)
{
  for (const auto & node : graph) {
    if (node.nodeid == std::numeric_limits<int>::max()) {  // Skip "deleted" nodes
      continue;
    }
    Json json_feature, json_properties, json_geometry, json_metadata, json_operations;
    json_geometry["type"] = "Point";
    json_geometry["coordinates"] = std::vector<float>{node.coords.x, node.coords.y};
    json_feature["geometry"] = json_geometry;
    json_properties["id"] = node.nodeid;
    json_properties["frame"] = node.coords.frame_id;
    convertMetaDataToJson(node.metadata, json_metadata);
    if (json_metadata.size() > 0) {
      json_properties["metadata"] = json_metadata;
    }
    convertOperationsToJson(node.operations, json_operations);
    if (json_operations.size() > 0) {
      json_properties["operations"] = json_operations;
    }
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
      Json json_edge, json_properties, json_geometry, json_metadata, json_operations;
      json_geometry["type"] = "MultiLineString";
      json_edge["geometry"] = json_geometry;
      json_properties["id"] = edge.edgeid;
      json_properties["startid"] = edge.start->nodeid;
      json_properties["endid"] = edge.end->nodeid;
      convertMetaDataToJson(edge.metadata, json_metadata);
      if (json_metadata.size() > 0) {
        json_properties["metadata"] = json_metadata;
      }
      convertOperationsToJson(edge.operations, json_operations);
      if (json_operations.size() > 0) {
        json_properties["operations"] = json_operations;
      }
      json_properties["cost"] = edge.edge_cost.cost;
      json_properties["overridable"] = edge.edge_cost.overridable;
      json_edge["properties"] = json_properties;
      json_edge["type"] = "Feature";
      json_edges.push_back(json_edge);
    }
  }
}

void GeoJsonGraphFileSaver::convertMetaDataToJson(
  const Metadata & metadata, Json & json_metadata)
{
  /* Function partially created using GPT */
  for (auto itr = metadata.data.begin(); itr != metadata.data.end(); itr++) {
    if (itr->second.type() == typeid(std::string)) {
      json_metadata[itr->first] = std::any_cast<std::string>(itr->second);
    } else if (itr->second.type() == typeid(int)) {
      json_metadata[itr->first] = std::any_cast<int>(itr->second);
    } else if (itr->second.type() == typeid(unsigned int)) {
      json_metadata[itr->first] = std::any_cast<unsigned int>(itr->second);
    } else if (itr->second.type() == typeid(float)) {
      json_metadata[itr->first] = std::any_cast<float>(itr->second);
    } else if (itr->second.type() == typeid(bool)) {
      json_metadata[itr->first] = std::any_cast<bool>(itr->second);
    } else if (itr->second.type() == typeid(Metadata)) {
      // If the itr->second is another Metadata, recursively convert it to JSON
      Json nested_metadata_json;
      convertMetaDataToJson(std::any_cast<Metadata>(itr->second), nested_metadata_json);
      json_metadata[itr->first] = nested_metadata_json;
    } else if (itr->second.type() == typeid(std::vector<std::any>)) {
      // If the itr->second is a vector, convert each element
      std::vector<Json> arrayJson;
      for (const auto & element : std::any_cast<std::vector<std::any>>(itr->second)) {
        if (element.type() == typeid(std::string)) {
          arrayJson.push_back(std::any_cast<std::string>(element));
        } else if (element.type() == typeid(int)) {
          arrayJson.push_back(std::any_cast<int>(element));
        } else if (element.type() == typeid(unsigned int)) {
          arrayJson.push_back(std::any_cast<unsigned int>(element));
        } else if (element.type() == typeid(float)) {
          arrayJson.push_back(std::any_cast<float>(element));
        } else if (element.type() == typeid(bool)) {
          arrayJson.push_back(std::any_cast<bool>(element));
        }
      }
      json_metadata[itr->first] = arrayJson;
    } else {
      // If we have an unknown type, handle as needed
      json_metadata[itr->first] = itr->second.type().name();
    }
  }
}

void GeoJsonGraphFileSaver::convertOperationsToJson(
  const Operations & operations, Json & json_operations)
{
  for (const auto & operation : operations) {
    Json json_operation, json_metadata;
    json_operation["type"] = operation.type;
    json_operation["trigger"] = operation.trigger;
    convertMetaDataToJson(operation.metadata, json_metadata);
    if (json_metadata.size()) {
      json_operation["metadata"] = json_metadata;
    }
    json_operations[operation.type] = json_operation;
  }
}
}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::GeoJsonGraphFileSaver, nav2_route::GraphFileSaver)
