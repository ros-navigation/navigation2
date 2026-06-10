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

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>

#include "nav2_route/plugins/graph_file_loaders/osm_graph_file_loader.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

void writeOsmToFile(const std::string & xml, const std::string & file_path)
{
  std::ofstream out(file_path);
  out << xml;
  out.close();
}

// A non-existent path must fail cleanly (false), never throw.
TEST(OsmGraphFileLoader, test_file_does_not_exist)
{
  Graph graph;
  GraphToIDMap graph_to_id_map;
  std::string file_path;
  OsmGraphFileLoader graph_file_loader;
  EXPECT_FALSE(graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path));
}

// Malformed XML must be caught by tinyxml2's LoadFile and reported as failure.
TEST(OsmGraphFileLoader, test_malformed_xml)
{
  const std::string file_path = "malformed.osm";
  writeOsmToFile("<osm><way></osm", file_path);
  Graph graph;
  GraphToIDMap graph_to_id_map;
  OsmGraphFileLoader graph_file_loader;
  EXPECT_FALSE(graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path));
  std::filesystem::remove(file_path);
}

// A well-formed file with nodes but no (kept) ways yields no graph -> failure.
TEST(OsmGraphFileLoader, test_no_ways)
{
  const std::string file_path = "no_ways.osm";
  const std::string xml =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<osm version=\"0.6\">\n"
    "  <node id=\"101\" lat=\"40.0\" lon=\"-75.0\"/>\n"
    "  <node id=\"102\" lat=\"40.0009\" lon=\"-75.0\"/>\n"
    "</osm>";
  writeOsmToFile(xml, file_path);
  Graph graph;
  GraphToIDMap graph_to_id_map;
  OsmGraphFileLoader graph_file_loader;
  EXPECT_FALSE(graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path));
  std::filesystem::remove(file_path);
}
