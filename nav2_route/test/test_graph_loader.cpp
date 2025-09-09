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
#include "tf2_ros/static_transform_broadcaster.h"

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

  EXPECT_THROW(GraphLoader graph_loader(node, tf, frame), std::runtime_error);
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
  EXPECT_TRUE(graph_loader.loadGraphFromParameter(graph, graph_to_id_map));
  EXPECT_FALSE(graph_loader.loadGraphFromFile(graph, graph_to_id_map, filepath));
}

TEST(GraphLoader, test_transformation_api)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_loader_test");
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
  EXPECT_TRUE(graph_loader.loadGraphFromFile(graph, graph_to_id_map, filepath));

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

  graph[0].coords.frame_id = "map_test";
  EXPECT_EQ(graph[0].coords.frame_id, "map_test");
  double or_coord = graph[0].coords.x;
  EXPECT_TRUE(graph_loader.transformGraph(graph));
  EXPECT_EQ(graph[0].coords.frame_id, "map");
  EXPECT_NE(graph[0].coords.x, or_coord);

  // Test failing to do so because no frame exists
  graph[0].coords.frame_id = "map_test2";
  EXPECT_EQ(graph[0].coords.frame_id, "map_test2");
  or_coord = graph[0].coords.x;
  EXPECT_FALSE(graph_loader.transformGraph(graph));
  EXPECT_EQ(graph[0].coords.frame_id, "map_test2");
  EXPECT_EQ(graph[0].coords.x, or_coord);
}

TEST(GraphLoader, test_transformation_api2)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("graph_loader_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);

  std::string frame = "map";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("nav2_route") +
      "/test/test_graphs/no_frame.json"));

  GraphLoader graph_loader(node, tf, frame);

  // Test with a file that has unknown frames that cannot be transformed
  Graph graph;
  GraphToIDMap graph_to_id_map;
  EXPECT_FALSE(graph_loader.loadGraphFromParameter(graph, graph_to_id_map));
  std::string filepath = ament_index_cpp::get_package_share_directory("nav2_route") +
    "/test/test_graphs/no_frame.json";
  EXPECT_FALSE(graph_loader.loadGraphFromFile(graph, graph_to_id_map, filepath));
}
