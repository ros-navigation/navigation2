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

#include <fstream>

#include "gtest/gtest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav2_route/graph_file_loader.hpp"

class RclCppFixture
{
 public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route; // NOLINT

TEST(GraphParser, test_graph_parser)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_parser_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  GraphFileLoader graph_file_loader(node, tf, "map");

  Graph graph;
  GraphToIDMap graph_to_id_map;
  graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, "map");

//  GraphFileLoader
  EXPECT_TRUE(true);
}