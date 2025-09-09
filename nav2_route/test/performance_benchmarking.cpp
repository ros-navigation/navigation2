// Copyright (c) 2025 Samsung Research
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

#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/route_planner.hpp"
#include "nav2_route/node_spatial_tree.hpp"

using namespace nav2_route;  // NOLINT

// This is a script to generate a regularized but arbitrarily sized graph for testing
// the route planner's performance across massive spaces

// Size of the benchmarking side length (e.g. 1000 x 1000 = 1,000,000 nodes)
const unsigned int DIM = 300;
// Number of tests to average results over
const unsigned int NUM_TESTS = 100;

inline Graph createGraph()
{
  Graph graph;
  graph.resize(DIM * DIM);

  EdgeCost e_cost;
  unsigned int curr_edge_idx = DIM * DIM + 1;

  unsigned int curr_graph_idx = 0;
  for (unsigned int j = 0; j != DIM; j++) {
    for (unsigned int i = 0; i != DIM; i++) {
      Node & node = graph[curr_graph_idx];
      node.nodeid = curr_graph_idx + 1;
      node.coords.x = i;
      node.coords.y = j;

      if (i > 0) {
        // (i - 1, j)
        node.addEdge(e_cost, &graph[curr_graph_idx - 1], curr_edge_idx++);
        graph[curr_graph_idx - 1].addEdge(e_cost, &node, curr_edge_idx++);
      }
      if (j > 0) {
        // (i, j - 1)
        node.addEdge(e_cost, &graph[curr_graph_idx - DIM], curr_edge_idx++);
        graph[curr_graph_idx - DIM].addEdge(e_cost, &node, curr_edge_idx++);
      }

      curr_graph_idx++;
    }
  }

  return graph;
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("route_benchmarking2");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  Graph graph = createGraph();

  // To visualize for smaller graphs reasonable for rviz to handle (DIM < 50)
  // auto graph_vis_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
  //   "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  // graph_vis_pub->publish(utils::toMsg(graph, "map", node->now()));

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  RoutePlanner planner;
  planner.configure(node, tf_buffer, costmap_subscriber);
  std::vector<unsigned int> blocked_ids;
  Route route;

  // // First test: Plan clear across the maximum diagonal
  // auto start = node->now();
  // for (unsigned int i = 0; i != NUM_TESTS; i++) {
  //   route = planner.findRoute(graph, 0u, static_cast<unsigned int>(DIM * DIM - 1), blocked_ids);
  // }
  // auto end = node->now();
  // RCLCPP_INFO(
  //   node->get_logger(),
  //   "Across map took %0.5f milliseconds.",
  //   (end - start).seconds() * 1000.0 / static_cast<double>(NUM_TESTS));
  // RCLCPP_INFO(
  //   node->get_logger(),
  //   "Route size: %li", route.edges.size());

  // // Second test: Random start and goal poses
  // srand (time(NULL));
  // unsigned int route_legs = 0;
  // start = node->now();
  // for (unsigned int i = 0; i != NUM_TESTS; i++) {
  //   unsigned int start = rand() % DIM;
  //   unsigned int goal = rand() % DIM;
  //   while (start == goal) {
  //     goal = rand() % DIM;
  //   }
  //   route = planner.findRoute(graph, start, goal, blocked_ids);
  //   route_legs += route.edges.size();
  // }
  // end = node->now();
  // RCLCPP_INFO(
  //   node->get_logger(),
  //   "Random planning took %0.5f milliseconds.",
  //   (end - start).seconds() * 1000.0 / static_cast<double>(NUM_TESTS));
  // RCLCPP_INFO(
  //   node->get_logger(),
  //   "Averaged route size: %f", static_cast<double>(route_legs) / static_cast<double>(NUM_TESTS))

  // Third test:
  // Check how much time it takes to do random lookups in the Kd-tree of various graph sizes
  std::shared_ptr<NodeSpatialTree> kd_tree = std::make_shared<NodeSpatialTree>();
  kd_tree->computeTree(graph);

  auto start = node->now();
  unsigned int seed = 1u;
  for (unsigned int i = 0; i != NUM_TESTS; i++) {
    std::vector<unsigned int> kd_tree_idxs;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<float>(rand_r(&seed) % DIM);
    pose.pose.position.y = static_cast<float>(rand_r(&seed) % DIM);
    kd_tree->findNearestGraphNodesToPose(pose, kd_tree_idxs);
  }
  auto end = node->now();
  RCLCPP_INFO(
    node->get_logger(),
    "Finding the nodes in the K-d tree took %0.5f milliseconds.",
    (end - start).seconds() * 1000.0 / static_cast<double>(NUM_TESTS));

  return 0;
}
