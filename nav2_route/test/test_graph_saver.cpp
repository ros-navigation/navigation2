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
// limitations under the License. Reserved.

#include <memory>
#include <string>
#include <fstream>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_route/graph_loader.hpp"
#include "nav2_route/graph_saver.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route; //NOLINT

TEST(GraphSaver, test_invalid_plugin)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_saver_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson"));

  // Set dummy parameter
  std::string default_plugin = "nav2_route::Dummy";
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_file_saver", rclcpp::ParameterValue(default_plugin));

  EXPECT_THROW(GraphSaver graph_saver(node, tf, frame), std::runtime_error);
}

TEST(GraphSaver, test_empty_filename)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_saver_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson"));

  GraphLoader graph_loader(node, tf, frame);
  GraphSaver graph_saver(node, tf, frame);

  Graph first_graph, second_graph;
  GraphToIDMap first_graph_to_id_map, second_graph_to_id_map;
  std::string file_path = "";
  graph_loader.loadGraphFromParameter(first_graph, first_graph_to_id_map);
  EXPECT_TRUE(graph_saver.saveGraphToFile(first_graph, file_path));
}

TEST(GraphSaver, test_api)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_saver_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson"));

  GraphLoader graph_loader(node, tf, frame);
  GraphSaver graph_saver(node, tf, frame);

  Graph first_graph, second_graph;
  GraphToIDMap first_graph_to_id_map, second_graph_to_id_map;
  std::string file_path = "test.geojson";
  graph_loader.loadGraphFromParameter(first_graph, first_graph_to_id_map);
  EXPECT_TRUE(graph_saver.saveGraphToFile(first_graph, file_path));
  graph_loader.loadGraphFromFile(second_graph, second_graph_to_id_map, file_path);
  EXPECT_EQ(first_graph.size(), second_graph.size());
  EXPECT_EQ(first_graph_to_id_map.size(), second_graph_to_id_map.size());
  for (size_t i = 0; i < first_graph.size(); ++i) {
    EXPECT_EQ(first_graph[i].nodeid, second_graph[i].nodeid);
    EXPECT_EQ(first_graph[i].coords.x, second_graph[i].coords.x);
    EXPECT_EQ(first_graph[i].coords.y, second_graph[i].coords.y);
  }
  for (size_t i = 0; i < first_graph_to_id_map.size(); ++i) {
    EXPECT_EQ(first_graph_to_id_map[i], second_graph_to_id_map[i]);
  }
  std::filesystem::remove(file_path);
}

TEST(GraphSaver, test_transformation_api)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_saver_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf);
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson"));

  GraphLoader graph_loader(node, tf, frame);

  // Test with a file that now works
  Graph graph;
  GraphToIDMap graph_to_id_map;
  std::string filepath;
  filepath =
    ament_index_cpp::get_package_share_directory("nav2_route") +
    "/graphs/aws_graph.geojson";
  graph_loader.loadGraphFromFile(graph, graph_to_id_map, filepath);

  // Test with another frame, should transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "map_test";
  transform.header.stamp = node->now();
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = 1.0;
  tf_broadcaster->sendTransform(transform);
  rclcpp::Rate(1).sleep();
  tf_broadcaster->sendTransform(transform);
  rclcpp::spin_all(node->get_node_base_interface(), std::chrono::milliseconds(50));

  GraphSaver graph_saver(node, tf, frame);
  std::string file_path = "test.geojson";
  graph[0].coords.frame_id = "map_test";
  EXPECT_EQ(graph[0].coords.frame_id, "map_test");
  double or_coord = graph[0].coords.x;
  EXPECT_TRUE(graph_saver.saveGraphToFile(graph, file_path));
  EXPECT_EQ(graph[0].coords.frame_id, "map");
  EXPECT_NE(graph[0].coords.x, or_coord);
  std::filesystem::remove(file_path);

  // Test failing to do so because no frame exists
  graph[0].coords.frame_id = "map_test2";
  EXPECT_EQ(graph[0].coords.frame_id, "map_test2");
  or_coord = graph[0].coords.x;
  EXPECT_FALSE(graph_saver.saveGraphToFile(graph, file_path));
  EXPECT_EQ(graph[0].coords.frame_id, "map_test2");
  EXPECT_EQ(graph[0].coords.x, or_coord);
}
