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

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>


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
  Node::initMotionModel(10);

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

  EXPECT_TRUE(true);
}

}  // namespace nav2_route
