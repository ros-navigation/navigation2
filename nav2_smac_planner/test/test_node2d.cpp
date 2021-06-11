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
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

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
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(&costmapA, 72);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // test construction
  unsigned char cost = static_cast<unsigned char>(1);
  nav2_smac_planner::Node2D testA(1);
  testA.setCost(cost);
  nav2_smac_planner::Node2D testB(1);
  testB.setCost(cost);
  EXPECT_EQ(testA.getCost(), 1.0f);
  nav2_smac_planner::SearchInfo info;
  info.cost_penalty = 1.0;
  unsigned int size = 10;
  nav2_smac_planner::Node2D::initMotionModel(
    nav2_smac_planner::MotionModel::MOORE, size, size, size, info);

  // test reset
  testA.reset();
  EXPECT_TRUE(std::isnan(testA.getCost()));

  // check collision checking
  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);
  testA.setCost(255);
  EXPECT_EQ(testA.isNodeValid(true, checker.get()), true);
  testA.setCost(10);

  // check traversal cost computation
  EXPECT_NEAR(testB.getTraversalCost(&testA), 1.03f, 0.1f);

  // check heuristic cost computation
  nav2_smac_planner::Node2D::Coordinates A(0.0, 0.0);
  nav2_smac_planner::Node2D::Coordinates B(10.0, 5.0);
  EXPECT_NEAR(testB.getHeuristicCost(A, B, nullptr), 15., 0.01);

  // check operator== works on index
  unsigned char costC = '2';
  nav2_smac_planner::Node2D testC(1);
  testC.setCost(costC);
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
  EXPECT_EQ(nav2_smac_planner::Node2D::getIndex(1u, 1u, 10u), 11u);
  EXPECT_EQ(nav2_smac_planner::Node2D::getIndex(6u, 43u, 10u), 436u);
  EXPECT_EQ(nav2_smac_planner::Node2D::getCoords(436u, 10u, 1u).x, 6u);
  EXPECT_EQ(nav2_smac_planner::Node2D::getCoords(436u, 10u, 1u).y, 43u);
  EXPECT_THROW(nav2_smac_planner::Node2D::getCoords(436u, 10u, 10u), std::runtime_error);
}

TEST(Node2DTest, test_node_2d_neighbors)
{
  nav2_smac_planner::SearchInfo info;
  unsigned int size_x = 10u;
  unsigned int size_y = 10u;
  unsigned int quant = 0u;
  // test neighborhood computation
  nav2_smac_planner::Node2D::initMotionModel(
    nav2_smac_planner::MotionModel::VON_NEUMANN, size_x,
    size_y, quant, info);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets.size(), 4u);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[0], -1);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[1], 1);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[2], -10);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[3], 10);

  size_x = 100u;
  nav2_smac_planner::Node2D::initMotionModel(
    nav2_smac_planner::MotionModel::MOORE, size_x, size_y,
    quant, info);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets.size(), 8u);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[0], -1);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[1], 1);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[2], -100);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[3], 100);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[4], -101);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[5], -99);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[6], 99);
  EXPECT_EQ(nav2_smac_planner::Node2D::_neighbors_grid_offsets[7], 101);

  nav2_costmap_2d::Costmap2D costmapA(10, 10, 0.05, 0.0, 0.0, 0);
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(&costmapA, 72);
  unsigned char cost = static_cast<unsigned int>(1);
  nav2_smac_planner::Node2D * node = new nav2_smac_planner::Node2D(1);
  node->setCost(cost);
  std::function<bool(const unsigned int &, nav2_smac_planner::Node2D * &)> neighborGetter =
    [&, this](const unsigned int & index, nav2_smac_planner::Node2D * & neighbor_rtn) -> bool
    {
      return false;
    };

  nav2_smac_planner::Node2D::NodeVector neighbors;
  node->getNeighbors(neighborGetter, checker.get(), false, neighbors);
  delete node;

  // should be empty since totally invalid
  EXPECT_EQ(neighbors.size(), 0u);
}
