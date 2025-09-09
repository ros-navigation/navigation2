// Copyright (c) 2025, Open Navigation LLC
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/path_converter.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

TEST(PathConverterTest, test_path_converter_api)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  nav_msgs::msg::Path path_msg;
  auto sub = node->create_subscription<nav_msgs::msg::Path>(
    "plan", rclcpp::QoS(10), [&, this](nav_msgs::msg::Path msg) {path_msg = msg;});

  PathConverter converter;
  converter.configure(node);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);
  Route route;
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 10;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;
  test_node2.nodeid = 11;
  test_node2.coords.x = 10.0;
  test_node2.coords.y = 10.0;
  test_node3.nodeid = 12;
  test_node3.coords.x = 20.0;
  test_node3.coords.y = 20.0;

  DirectionalEdge test_edge1, test_edge2;
  test_edge1.edgeid = 13;
  test_edge1.start = &test_node1;
  test_edge1.end = &test_node2;
  test_edge2.edgeid = 14;
  test_edge2.start = &test_node2;
  test_edge2.end = &test_node3;

  route.start_node = &test_node1;
  route.route_cost = 50.0;
  route.edges.push_back(&test_edge1);
  route.edges.push_back(&test_edge2);
  ReroutingState info;

  auto path = converter.densify(route, info, frame, time);
  EXPECT_EQ(path.header.frame_id, frame);
  EXPECT_EQ(path.header.stamp.nanosec, time.nanoseconds());

  // 2 * sqrt(200) * 20 (0.05 density/m) + 1 (for starting node)
  EXPECT_EQ(path.poses.size(), 567u);
  EXPECT_NEAR(path.poses[0].pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(path.poses[0].pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(path.poses.back().pose.position.x, 20.0, 0.01);
  EXPECT_NEAR(path.poses.back().pose.position.y, 20.0, 0.01);

  rclcpp::Rate r(10);
  r.sleep();

  // Checks the same as returned and actually was published
  EXPECT_EQ(path_msg.poses.size(), path.poses.size());
  node_thread.reset();
}

TEST(PathConverterTest, test_path_single_pt_path)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);

  Node test_node;
  test_node.nodeid = 17;
  test_node.coords.x = 10.0;
  test_node.coords.y = 40.0;

  Route route;
  route.start_node = &test_node;
  ReroutingState info;

  auto path = converter.densify(route, info, frame, time);
  EXPECT_EQ(path.poses.size(), 1u);
  EXPECT_NEAR(path.poses[0].pose.position.x, 10.0, 0.01);
  EXPECT_NEAR(path.poses[0].pose.position.y, 40.0, 0.01);
}

TEST(PathConverterTest, test_prev_info_path)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);

  Node test_node;
  test_node.nodeid = 17;
  test_node.coords.x = 1.0;
  test_node.coords.y = 0.0;

  Route route;
  route.start_node = &test_node;

  DirectionalEdge edge;
  edge.end = &test_node;

  ReroutingState info;
  info.closest_pt_on_edge.x = 0.0;
  info.closest_pt_on_edge.y = 0.0;
  info.curr_edge = &edge;

  auto path = converter.densify(route, info, frame, time);
  EXPECT_EQ(path.poses.size(), 21u);  // 20 for density + 1 for single node point
}

TEST(PathConverterTest, test_path_converter_interpolation)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  float x0 = 10.0, y0 = 10.0, x1 = 20.0, y1 = 20.0;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  converter.interpolateEdge(x0, y0, x1, y1, poses);

  EXPECT_EQ(poses.size(), 283u);  // regular density + edges
  for (unsigned int i = 0; i != poses.size() - 1; i++) {
    // Check its always closer than the requested density
    EXPECT_LT(
      hypotf(
        poses[i].pose.position.x - poses[i + 1].pose.position.x,
        poses[i].pose.position.y - poses[i + 1].pose.position.y), 0.05);
  }
}

TEST(PathConverterTest, test_path_converter_zero_length_edge)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  float x0 = 10.0, y0 = 10.0, x1 = 10.0, y1 = 10.0;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  converter.interpolateEdge(x0, y0, x1, y1, poses);
  ASSERT_TRUE(poses.empty());
}
