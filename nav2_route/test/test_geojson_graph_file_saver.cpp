// Copyright (c) 2025 John Chrosniak
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

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_util/node_utils.hpp"
#include "nav2_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "nav2_route/plugins/graph_file_savers/geojson_graph_file_saver.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route; // NOLINT
using Json = nlohmann::json;

Json g_simple_graph = nlohmann::json::parse(
  R"(
  {
    "features": [
    {
      "type": "Feature",
      "properties":
      {
        "id": 0,
        "frame": "map"
      },
      "geometry":
      { "type": "Point",
        "coordinates": [ 0.0, 0.0 ]
      }
    },
    {
      "type": "Feature",
      "properties":
      {
        "id": 1,
        "frame": "map"
      },
      "geometry":
      {
        "type":"Point",
        "coordinates": [ 1.0, 0.0 ]
      }
    },
    {
      "type": "Feature",
      "properties":
      {
        "id": 2,
        "startid": 0,
        "endid": 1
      },
      "geometry":
      {
        "type": "MultiLineString",
        "coordinates": [ [ [ 0.0, 0.0 ], [ 1.0, 0.0 ] ] ]
      }
    }
  ]
  }
  )");

void writeGraphToFile(const Json & graph, const std::string & file_path = "graph.geojson")
{
  std::ofstream missing_id_file(file_path);
  missing_id_file << graph;
  missing_id_file.close();
}

TEST(GeoJsonGraphFileSaver, test_file_empty)
{
  Graph graph;
  std::string file_path;

  GeoJsonGraphFileSaver graph_file_saver;
  bool result = graph_file_saver.saveGraphToFile(graph, file_path);
  EXPECT_FALSE(result);
}

TEST(GeoJsonGraphFileSaver, test_node_metadata) {
  Json json_graph = g_simple_graph;

  Json node_metadata = nlohmann::json::parse(
    R"(
    {
      "metadata":
      {
        "person":
        {
          "name": "josh",
          "can_drive": true,
          "top_speed": 0.85,
          "age": 29,
          "altitude": -10
        },
        "double_array": [0.0, 1.0, 2.0],
        "int_array": [0, 1, 2],
        "string_array": ["a", "b", "c"],
        "bool_array": [true, false, true]
      }
    }
)");
  Json edge_metadata = nlohmann::json::parse(
    R"(
    {
      "metadata":
      {
        "color": "green"
      }
    }
)");

  json_graph["features"][0]["properties"].insert(node_metadata.begin(), node_metadata.end());
  json_graph["features"][2]["properties"].insert(edge_metadata.begin(), edge_metadata.end());

  std::string file_path = "metadata.geojson";
  writeGraphToFile(json_graph, file_path);

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  GeoJsonGraphFileSaver graph_file_saver;
  bool result = graph_file_saver.saveGraphToFile(graph, file_path);
  EXPECT_TRUE(result);
  Graph graph2;
  GraphToIDMap graph_to_id_map2;
  graph_file_loader.loadGraphFromFile(graph2, graph_to_id_map2, file_path);
  EXPECT_EQ(graph.size(), graph2.size());
  for (size_t i = 0; i < graph.size(); ++i) {
    EXPECT_EQ(graph[i].nodeid, graph2[i].nodeid);
    EXPECT_EQ(graph[i].coords.x, graph2[i].coords.x);
    EXPECT_EQ(graph[i].coords.y, graph2[i].coords.y);
    EXPECT_EQ(graph[i].coords.frame_id, graph2[i].coords.frame_id);
    EXPECT_EQ(graph[i].neighbors.size(), graph2[i].neighbors.size());
    for (size_t j = 0; j < graph[i].neighbors.size(); ++j) {
      EXPECT_EQ(graph[i].neighbors[j].edgeid, graph2[i].neighbors[j].edgeid);
      EXPECT_EQ(graph[i].neighbors[j].start->nodeid, graph2[i].neighbors[j].start->nodeid);
      EXPECT_EQ(graph[i].neighbors[j].end->nodeid, graph2[i].neighbors[j].end->nodeid);
      EXPECT_EQ(graph[i].neighbors[j].edge_cost.cost, graph2[i].neighbors[j].edge_cost.cost);
      EXPECT_EQ(
        graph[i].neighbors[j].edge_cost.overridable,
        graph2[i].neighbors[j].edge_cost.overridable);
    }
  }
  EXPECT_EQ(graph_to_id_map.size(), graph_to_id_map2.size());
  for (const auto & pair : graph_to_id_map) {
    EXPECT_EQ(pair.first, graph_to_id_map2[pair.second]);
  }
  // Check node metadata
  Metadata metadata;
  metadata = graph2[0].metadata.getValue("person", metadata);

  std::string name;
  name = metadata.getValue("name", name);
  EXPECT_EQ(name, node_metadata["metadata"]["person"]["name"]);

  bool can_drive = false;
  can_drive = metadata.getValue("can_drive", can_drive);
  EXPECT_EQ(can_drive, node_metadata["metadata"]["person"]["can_drive"]);

  float top_speed = 0.0f;
  top_speed = metadata.getValue("top_speed", top_speed);
  EXPECT_NEAR(top_speed, node_metadata["metadata"]["person"]["top_speed"], 1e-6);

  unsigned int age = 0;
  age = metadata.getValue("age", age);
  EXPECT_EQ(age, node_metadata["metadata"]["person"]["age"]);

  std::vector<std::any> array;
  array = graph2[0].metadata.getValue("double_array", array);
  EXPECT_EQ(array.size(), 3u);

  EXPECT_EQ(std::any_cast<float>(array[0]), 0.0f);
  EXPECT_EQ(std::any_cast<float>(array[1]), 1.0f);
  EXPECT_EQ(std::any_cast<float>(array[2]), 2.0f);

  // Check edge metadata
  std::string color;
  color = graph2[0].neighbors[0].metadata.getValue("color", color);
  EXPECT_EQ(color, edge_metadata["metadata"]["color"]);
  std::filesystem::remove(file_path);
}

TEST(GeoJsonGraphFileSaver, operations)
{
  Json json_graph = g_simple_graph;

  Json json_operation = nlohmann::json::parse(
    R"(
    {
      "operations":
      {
        "open_door":
        {
            "type": "open_door",
            "trigger": "ON_EXIT",
            "metadata":
            {
              "color": "green"
            }
        }
      }
    }
)");

  json_graph["features"][0]["properties"].insert(json_operation.begin(), json_operation.end());

  std::string file_path = "operations.geojson";
  writeGraphToFile(json_graph, file_path);

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  GeoJsonGraphFileSaver graph_file_saver;
  Graph graph2;
  GraphToIDMap graph_to_id_map2;
  bool result = graph_file_saver.saveGraphToFile(graph, file_path);
  EXPECT_TRUE(result);
  graph_file_loader.loadGraphFromFile(graph2, graph_to_id_map2, file_path);
  EXPECT_EQ(graph.size(), graph2.size());
  for (size_t i = 0; i < graph.size(); ++i) {
    EXPECT_EQ(graph[i].nodeid, graph2[i].nodeid);
    EXPECT_EQ(graph[i].coords.x, graph2[i].coords.x);
    EXPECT_EQ(graph[i].coords.y, graph2[i].coords.y);
    EXPECT_EQ(graph[i].coords.frame_id, graph2[i].coords.frame_id);
    EXPECT_EQ(graph[i].neighbors.size(), graph2[i].neighbors.size());
    for (size_t j = 0; j < graph[i].neighbors.size(); ++j) {
      EXPECT_EQ(graph[i].neighbors[j].edgeid, graph2[i].neighbors[j].edgeid);
      EXPECT_EQ(graph[i].neighbors[j].start->nodeid, graph2[i].neighbors[j].start->nodeid);
      EXPECT_EQ(graph[i].neighbors[j].end->nodeid, graph2[i].neighbors[j].end->nodeid);
      EXPECT_EQ(graph[i].neighbors[j].edge_cost.cost, graph2[i].neighbors[j].edge_cost.cost);
      EXPECT_EQ(
        graph[i].neighbors[j].edge_cost.overridable,
        graph2[i].neighbors[j].edge_cost.overridable);
    }
  }
  EXPECT_EQ(graph_to_id_map.size(), graph_to_id_map2.size());
  for (const auto & pair : graph_to_id_map) {
    EXPECT_EQ(pair.first, graph_to_id_map2[pair.second]);
  }
  std::filesystem::remove(file_path);

  // Check node operations
  Operations node_operations = graph2[0].operations;

  EXPECT_EQ(node_operations.size(), 1u);

  EXPECT_EQ(node_operations[0].type, "open_door");
  EXPECT_EQ(node_operations[0].trigger, OperationTrigger::ON_EXIT);

  std::string color;
  color = node_operations[0].metadata.getValue("color", color);
  EXPECT_EQ(color, "green");
}

TEST(GeoJsonGraphFileSaver, simple_graph)
{
  std::string file_path = "simple_graph.geojson";

  writeGraphToFile(g_simple_graph, file_path);

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  GeoJsonGraphFileSaver graph_file_saver;
  bool result = graph_file_saver.saveGraphToFile(graph, file_path);
  EXPECT_TRUE(result);
  Graph graph2;
  GraphToIDMap graph_to_id_map2;
  graph_file_loader.loadGraphFromFile(graph2, graph_to_id_map2, file_path);
  std::filesystem::remove(file_path);

  // Node 1
  EXPECT_EQ(graph2[0].nodeid, 0u);
  EXPECT_EQ(graph2[0].coords.x, 0.0);
  EXPECT_EQ(graph2[0].coords.x, 0.0);
  EXPECT_EQ(graph2[0].coords.frame_id, "map");
  EXPECT_EQ(graph2[0].neighbors.size(), 1u);
  EXPECT_EQ(graph_to_id_map2[0], graph2[0].nodeid);

  // Node 2
  EXPECT_EQ(graph2[1].nodeid, 1u);
  EXPECT_EQ(graph2[1].coords.x, 1.0);
  EXPECT_EQ(graph2[1].coords.y, 0.0);
  EXPECT_EQ(graph2[1].coords.frame_id, "map");
  EXPECT_EQ(graph2[1].neighbors.size(), 0u);
  EXPECT_EQ(graph_to_id_map2[1], graph2[1].nodeid);

  // Edge from node 1 -> 2
  EXPECT_EQ(graph2[0].neighbors[0].edgeid, 2u);
  EXPECT_EQ(graph2[0].neighbors[0].start, &graph2[0]);
  EXPECT_EQ(graph2[0].neighbors[0].end, &graph2[1]);
  EXPECT_TRUE(graph2[0].neighbors[0].edge_cost.overridable);
  EXPECT_EQ(graph2[0].neighbors[0].edge_cost.cost, 0.0f);
}

TEST(GeoJsonGraphFileSaver, sample_graph)
{
  auto file_path = ament_index_cpp::get_package_share_directory("nav2_route") +
    "/graphs/sample_graph.geojson";

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  GeoJsonGraphFileSaver graph_file_saver;
  bool result = graph_file_saver.saveGraphToFile(graph, file_path);
  EXPECT_TRUE(result);
  Graph graph2;
  GraphToIDMap graph_to_id_map2;
  graph_file_loader.loadGraphFromFile(graph2, graph_to_id_map2, file_path);

  Metadata region;
  region = graph2[0].metadata.getValue("region", region);
  EXPECT_EQ(region.data.size(), 3u);

  std::vector<std::any> x_values;
  x_values = region.getValue("x_values", x_values);
  EXPECT_EQ(x_values.size(), 4u);

  EXPECT_EQ(graph2[0].neighbors[0].edge_cost.cost, 10.0f);
  EXPECT_EQ(graph2[0].neighbors[0].edge_cost.overridable, false);

  auto & operations = graph2[0].neighbors[0].operations;

  EXPECT_EQ(operations[0].type, "open_door");

  std::string type;
  type = operations[1].metadata.getValue("type", type);
  EXPECT_EQ(type, "jpg");
}
