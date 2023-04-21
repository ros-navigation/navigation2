// Copyright (c) 2023 Samsung Research America
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

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_route/node.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

namespace nav2_route
{

TEST(test_node, node_test) {
  Node::initMotionModel(10, 15);

  // Check State visiting
  Node node(10);

  EXPECT_FALSE(node.wasVisited());
  EXPECT_FALSE(node.isQueued());

  node.queue();
  EXPECT_TRUE(node.isQueued());
  EXPECT_FALSE(node.wasVisited());

  node.visit();
  EXPECT_TRUE(node.wasVisited());
  EXPECT_FALSE(node.isQueued());

  // Check static index functions
  EXPECT_EQ(Node::getIndex(1u, 1u, 10u), 11u);
  EXPECT_EQ(Node::getIndex(6u, 42, 10u), 426u);
  EXPECT_EQ(Node::getCoords(10u).x, 0.0);
  EXPECT_EQ(Node::getCoords(10u).y, 1.0);
}

TEST(test_node, test_node_neighbors)
{
  int x_size = 100;
  int y_size = 40;
  Node::initMotionModel(x_size, y_size);

  EXPECT_EQ(Node::neighbors_grid_offsets[0], -1);
  EXPECT_EQ(Node::neighbors_grid_offsets[1], 1);
  EXPECT_EQ(Node::neighbors_grid_offsets[2], -x_size);
  EXPECT_EQ(Node::neighbors_grid_offsets[3], x_size);
  EXPECT_EQ(Node::neighbors_grid_offsets[4], -x_size - 1);
  EXPECT_EQ(Node::neighbors_grid_offsets[5], -x_size + 1);
  EXPECT_EQ(Node::neighbors_grid_offsets[6], +x_size - 1);
  EXPECT_EQ(Node::neighbors_grid_offsets[7], +x_size + 1);

  std::unordered_map<unsigned int, Node> graph;

  graph.emplace(0, Node(0));

  Node::NodeGetter neighbor_getter =
    [&, this](const unsigned int & index, Node::NodePtr & neighbor_rtn)
    {
      auto iter = graph.find(index);

      if (iter != graph.end()) {
        neighbor_rtn = &(iter->second);
      }

      neighbor_rtn = &(graph.emplace(index, Node(index)).first->second);
    };

  nav2_costmap_2d::Costmap2D costmap(x_size, y_size, 0.0, 0.0, 1);
  CollisionChecker collision_checker(&costmap);

  Node::NodePtr start;
  neighbor_getter(0, start);

  Node::NodeVector neighbors;

  start->getNeighbors(neighbor_getter, &collision_checker, false, neighbors);

  EXPECT_EQ(neighbors.size(), 3u);

  for (const auto neighbor : neighbors) {
    auto coords = Node::getCoords(neighbor->getIndex());

    EXPECT_TRUE(coords.x >= 0.0);
    EXPECT_TRUE(coords.y >= 0.0);
  }
}

TEST(test_collision_checker, collision_checker_test)
{
  // Create a dummy costmap
  nav2_costmap_2d::Costmap2D costmap(10u, 10u, 1, 0, 0);
  costmap.setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);

  CollisionChecker collision_checker(&costmap);
  EXPECT_TRUE(collision_checker.inCollision(0u, true));
}

}  // namespace nav2_route
