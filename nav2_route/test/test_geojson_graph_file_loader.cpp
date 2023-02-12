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

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

#include "nav2_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route; // NOLINT
using Json = nlohmann::json;

TEST(GeoJsonGraphFileLoader, file_does_not_exist)
{
  Graph graph;
  GraphToIDMap graph_to_id_map;
  std::string file_path = "graph.geojson";

  GeoJsonGraphFileLoader graph_file_loader;
  bool result = graph_file_loader.fileExists(file_path);
  EXPECT_FALSE(result);
}

TEST(GeoJsonGraphFileLoader, no_nodes_in_graph)
{
  Json json_obj = nlohmann::json::parse(
    R"(
  {
    "features": [
    { "type": "Feature",
      "properties":
      {
        "id": 123,
        "startid": 20,
        "endid": 4
      },
      "geometry":
      {
        "type": "MultiLineString",
        "coordinates": [ [ [ 1.0, 3.65 ], [ -4.0, 3.65 ] ] ]
      }
    }
  ]
  }
  )");
  std::string file_path = "no_nodes.geojson";

  std::ofstream missing_id_file(file_path);
  missing_id_file << json_obj;
  missing_id_file.close();

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  bool result = graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  EXPECT_FALSE(result);
  std::filesystem::remove(file_path);
}

TEST(GeoJsonGraphFileLoader, no_edges_in_graph)
{
  Json json_obj = nlohmann::json::parse(
    R"(
  {
    "features": [
    { "type": "Feature",
      "properties":
      {
        "frame": "map"
      },
      "geometry":
      {
        "type": "Point",
        "coordinates": [ 0.0, 0.0 ]
      }
    }
  ]
  }
  )");
  std::string file_path = "no_edges.geojson";

  std::ofstream missing_id_file(file_path);
  missing_id_file << json_obj;
  missing_id_file.close();

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  bool result = graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  EXPECT_FALSE(result);
  std::filesystem::remove(file_path);
}

TEST(GeoJsonGraphFileLoader, missing_node_id)
{
  Json json_obj = nlohmann::json::parse(
    R"(
  {
    "features": [
    { "type": "Feature",
      "properties":
      {
        "frame": "map"
      },
      "geometry":
      {
        "type": "Point",
        "coordinates": [ 0.0, 0.0 ]
      }
    },
    { "type": "Feature",
      "properties":
      {
        "frame": "map"
      },
      "geometry":
      {
        "type": "Point",
        "coordinates": [ 0.0, 0.0 ]
      }
    },
    { "type": "Feature",
      "properties":
      {
        "id": 123,
        "startid": 20,
        "endid": 4
      },
      "geometry":
      {
        "type": "MultiLineString",
        "coordinates": [ [ [ 1.0, 3.65 ], [ -4.0, 3.65 ] ] ]
      }
    }
  ]
  }
  )");
  std::string file_path = "missing_id.geojson";

  std::ofstream missing_id_file(file_path);
  missing_id_file << json_obj;
  missing_id_file.close();

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  EXPECT_THROW(
    graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path),
    Json::exception);
  std::filesystem::remove(file_path);
}

TEST(GeoJsonGraphFileLoader, start_id_does_not_exist)
{
  Json json_obj = nlohmann::json::parse(
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
        "id": 2,
        "startid": 10,
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

  std::string file_path = "invalid_start_id.geojson";

  std::ofstream missing_id_file(file_path);
  missing_id_file << json_obj;
  missing_id_file.close();

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  EXPECT_THROW(
    graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path),
    std::runtime_error);
}

TEST(GeoJsonGraphFileLoader, simple_graph)
{
  Json json_obj = nlohmann::json::parse(
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
  std::string file_path = "simple_graph.geojson";

  std::ofstream missing_id_file(file_path);
  missing_id_file << json_obj;
  missing_id_file.close();

  Graph graph;
  GraphToIDMap graph_to_id_map;
  GeoJsonGraphFileLoader graph_file_loader;
  bool result = graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path);
  EXPECT_TRUE(result);

  EXPECT_EQ(graph.size(), 2u);

  // Node 1
  EXPECT_EQ(graph[0].nodeid, 0u);
  EXPECT_EQ(graph[0].coords.x, 0.0);
  EXPECT_EQ(graph[0].coords.x, 0.0);
  EXPECT_EQ(graph[0].coords.frame_id, "map");
  EXPECT_EQ(graph[0].neighbors.size(), 1u);
  EXPECT_EQ(graph_to_id_map[0], graph[0].nodeid);

  // Node 2
  EXPECT_EQ(graph[1].nodeid, 1u);
  EXPECT_EQ(graph[1].coords.x, 1.0);
  EXPECT_EQ(graph[1].coords.y, 0.0);
  EXPECT_EQ(graph[1].coords.frame_id, "map");
  EXPECT_EQ(graph[1].neighbors.size(), 0u);
  EXPECT_EQ(graph_to_id_map[1], graph[1].nodeid);

  // Edge from node 1 -> 2
  EXPECT_EQ(graph[0].neighbors[0].edgeid, 2u);
  EXPECT_EQ(graph[0].neighbors[0].start, &graph[0]);
  EXPECT_EQ(graph[0].neighbors[0].end, &graph[1]);
  EXPECT_TRUE(graph[0].neighbors[0].edge_cost.overridable);
  EXPECT_EQ(graph[0].neighbors[0].edge_cost.cost, 0.0f);

  std::filesystem::remove(file_path);
}
