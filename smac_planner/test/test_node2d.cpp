// Copyright (c) 2020, Samsung Research America
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
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "smac_planner/node_2d.hpp"
#include "smac_planner/collision_checker.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(Node2DTest, test_node_2d)
{
  nav2_costmap_2d::Costmap2D costmapA(10, 10, 0.05, 0.0, 0.0, 0);
  smac_planner::GridCollisionChecker checker(&costmapA);

  // test construction
  unsigned char cost = static_cast<unsigned char>(1);
  smac_planner::Node2D testA(cost, 1);
  smac_planner::Node2D testB(cost, 1);
  EXPECT_EQ(testA.getCost(), 1.0f);

  // test reset
  testA.reset(10);
  EXPECT_EQ(testA.getCost(), 10.0f);

  // Check constants
  EXPECT_EQ(testA.neutral_cost, 50.0f);

  // check collision checking
  EXPECT_EQ(testA.isNodeValid(false, checker), true);
  testA.reset(254);
  EXPECT_EQ(testA.isNodeValid(false, checker), false);
  testA.reset(255);
  EXPECT_EQ(testA.isNodeValid(true, checker), true);
  EXPECT_EQ(testA.isNodeValid(false, checker), false);
  testA.reset(10);

  // check traversal cost computation
  EXPECT_EQ(testB.getTraversalCost(&testA), 58.0f);

  // check heuristic cost computation
  smac_planner::Node2D::Coordinates A(0.0, 0.0);
  smac_planner::Node2D::Coordinates B(10.0, 5.0);
  EXPECT_NEAR(testB.getHeuristicCost(A, B), 559.016, 0.01);

  // check operator== works on index
  unsigned char costC = '2';
  smac_planner::Node2D testC(costC, 1);
  EXPECT_TRUE(testA == testC);

  // check accumulated costs are set
  testC.setAccumulatedCost(100);
  EXPECT_EQ(testC.getAccumulatedCost(), 100.0f);

  // check visiting state
  EXPECT_EQ(testC.wasVisited(), false);
  testC.queued();
  EXPECT_EQ(testC.isQueued(), true);
  testC.visited();
  EXPECT_EQ(testC.wasVisited(), true);
  EXPECT_EQ(testC.isQueued(), false);

  // check index
  EXPECT_EQ(testC.getIndex(), 1u);

  // check static index functions
  EXPECT_EQ(smac_planner::Node2D::getIndex(1u, 1u, 10u), 11u);
  EXPECT_EQ(smac_planner::Node2D::getIndex(6u, 43u, 10u), 436u);
  EXPECT_EQ(smac_planner::Node2D::getCoords(436u, 10u, 1u).x, 6u);
  EXPECT_EQ(smac_planner::Node2D::getCoords(436u, 10u, 1u).y, 43u);
  EXPECT_THROW(smac_planner::Node2D::getCoords(436u, 10u, 10u), std::runtime_error);
}

TEST(Node2DTest, test_node_2d_neighbors)
{
  // test neighborhood computation
  smac_planner::Node2D::initNeighborhood(10u, smac_planner::MotionModel::VON_NEUMANN);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets.size(), 4u);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[0], -1);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[1], 1);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[2], -10);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[3], 10);

  smac_planner::Node2D::initNeighborhood(100u, smac_planner::MotionModel::MOORE);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets.size(), 8u);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[0], -1);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[1], 1);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[2], -100);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[3], 100);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[4], -101);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[5], -99);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[6], 99);
  EXPECT_EQ(smac_planner::Node2D::_neighbors_grid_offsets[7], 101);

  nav2_costmap_2d::Costmap2D costmapA(10, 10, 0.05, 0.0, 0.0, 0);
  smac_planner::GridCollisionChecker checker(&costmapA);
  unsigned char cost = static_cast<unsigned int>(1);
  smac_planner::Node2D * node = new smac_planner::Node2D(cost, 1);
  std::function<bool(const unsigned int &, smac_planner::Node2D * &)> neighborGetter =
    [&, this](const unsigned int & index, smac_planner::Node2D * & neighbor_rtn) -> bool
    {
      return true;
    };

  smac_planner::Node2D::NodeVector neighbors;
  smac_planner::Node2D::getNeighbors(node, neighborGetter, checker, false, neighbors);
  delete node;

  // should be empty since totally invalid
  EXPECT_EQ(neighbors.size(), 0u);
}
