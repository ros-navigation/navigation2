// Copyright (c) 2025, Joshua Wallace
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
// limitations under the License. Reserved.

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_route/graph_loader.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route; //NOLINT

TEST(GraphLoader, test_invalid_plugin)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_loader_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson"));

  // Set dummy parameter
  std::string default_plugin = "nav2_route::Dummy";
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_file_loader", rclcpp::ParameterValue(default_plugin));

  EXPECT_THROW(GraphLoader graph_loader(node, tf, frame), pluginlib::PluginlibException);
}

TEST(GraphLoader, test_api)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_loader_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson"));

  GraphLoader graph_loader(node, tf, frame);

  Graph graph;
  GraphToIDMap graph_to_id_map;
  std::string filepath;

  EXPECT_TRUE(graph_loader.loadGraphFromFile(graph, graph_to_id_map, filepath));
}
