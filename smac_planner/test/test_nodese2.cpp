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

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "smac_planner/node_se2.hpp"
#include "smac_planner/collision_checker.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NodeSE2Test, test_node_se2)
{
  smac_planner::SearchInfo info;
  info.change_penalty = 1.2;
  info.non_straight_penalty = 1.4;
  info.reverse_penalty = 2.1;
  info.minimum_turning_radius = 0.20;
  unsigned int size_x = 10;
  unsigned int size_y = 10;
  unsigned int size_theta = 72;

  smac_planner::NodeSE2::initMotionModel(
    smac_planner::MotionModel::DUBIN, size_x, size_y, size_theta, info);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    10, 10, 0.05, 0.0, 0.0, 0);
  smac_planner::GridCollisionChecker checker(costmapA);
  checker.setFootprint(nav2_costmap_2d::Footprint(), true);

  // test construction
  smac_planner::NodeSE2 testA(49);
  smac_planner::NodeSE2 testB(49);
  EXPECT_TRUE(std::isnan(testA.getCost()));

  // test node valid and cost
  testA.pose.x = 5;
  testA.pose.y = 5;
  testA.pose.theta = 0;
  EXPECT_EQ(testA.isNodeValid(true, checker), true);
  EXPECT_EQ(testA.isNodeValid(false, checker), true);
  EXPECT_EQ(testA.getCost(), 0.0f);

  // test reset
  testA.reset();
  EXPECT_TRUE(std::isnan(testA.getCost()));

  // Check constants
  EXPECT_EQ(testA.neutral_cost, sqrt(2));

  // check collision checking
  EXPECT_EQ(testA.isNodeValid(false, checker), true);

  // check traversal cost computation
  // simulated first node, should return neutral cost
  EXPECT_NEAR(testB.getTraversalCost(&testA), sqrt(2), 0.01);
  // now with straight motion, cost is 0, so will be sqrt(2) as well
  testB.setMotionPrimitiveIndex(1);
  testA.setMotionPrimitiveIndex(0);
  EXPECT_NEAR(testB.getTraversalCost(&testA), sqrt(2), 0.01);
  // same direction as parent, testB
  testA.setMotionPrimitiveIndex(1);
  EXPECT_NEAR(testB.getTraversalCost(&testA), 1.9799f, 0.01);
  // opposite direction as parent, testB
  testA.setMotionPrimitiveIndex(2);
  EXPECT_NEAR(testB.getTraversalCost(&testA), 3.67696f, 0.01);

  // will throw because never collision checked testB
  EXPECT_THROW(testA.getTraversalCost(&testB), std::runtime_error);

  // check motion primitives
  EXPECT_EQ(testA.getMotionPrimitiveIndex(), 2u);

  // check heuristic cost computation
  smac_planner::NodeSE2::computeWavefrontHeuristic(
    costmapA,
    static_cast<unsigned int>(10.0),
    static_cast<unsigned int>(5.0),
    0.0, 0.0);
  smac_planner::NodeSE2::Coordinates A(0.0, 0.0, 4.2);
  smac_planner::NodeSE2::Coordinates B(10.0, 5.0, 54.1);
  EXPECT_NEAR(testB.getHeuristicCost(B, A), 16.723, 0.01);

  // check operator== works on index
  smac_planner::NodeSE2 testC(49);
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
  EXPECT_EQ(testC.getIndex(), 49u);

  // check set pose and pose
  testC.setPose(smac_planner::NodeSE2::Coordinates(10.0, 5.0, 4));
  EXPECT_EQ(testC.pose.x, 10.0);
  EXPECT_EQ(testC.pose.y, 5.0);
  EXPECT_EQ(testC.pose.theta, 4);

  // check static index functions
  EXPECT_EQ(smac_planner::NodeSE2::getIndex(1u, 1u, 4u, 10u, 72u), 796u);
  EXPECT_EQ(smac_planner::NodeSE2::getCoords(796u, 10u, 72u).x, 1u);
  EXPECT_EQ(smac_planner::NodeSE2::getCoords(796u, 10u, 72u).y, 1u);
  EXPECT_EQ(smac_planner::NodeSE2::getCoords(796u, 10u, 72u).theta, 4u);

  delete costmapA;
}

TEST(NodeSE2Test, test_node_2d_neighbors)
{
  smac_planner::SearchInfo info;
  info.change_penalty = 1.2;
  info.non_straight_penalty = 1.4;
  info.reverse_penalty = 2.1;
  info.minimum_turning_radius = 4;  // 0.2 in grid coordinates
  unsigned int size_x = 100;
  unsigned int size_y = 100;
  unsigned int size_theta = 72;
  smac_planner::NodeSE2::initMotionModel(
    smac_planner::MotionModel::DUBIN, size_x, size_y, size_theta, info);


  // test neighborhood computation
  EXPECT_EQ(smac_planner::NodeSE2::motion_table.projections.size(), 3u);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[0]._x, 1.731517, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[0]._y, 0, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[0]._theta, 0, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[1]._x, 1.69047, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[1]._y, 0.3747, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[1]._theta, 5, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[2]._x, 1.69047, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[2]._y, -0.3747, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[2]._theta, -5, 0.01);

  smac_planner::NodeSE2::initMotionModel(
    smac_planner::MotionModel::REEDS_SHEPP, size_x, size_y, size_theta, info);

  EXPECT_EQ(smac_planner::NodeSE2::motion_table.projections.size(), 6u);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[0]._x, 1.731517, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[0]._y, 0, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[0]._theta, 0, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[1]._x, 1.69047, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[1]._y, 0.3747, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[1]._theta, 5, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[2]._x, 1.69047, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[2]._y, -0.3747, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[2]._theta, -5, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[3]._x, -1.731517, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[3]._y, 0, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[3]._theta, 0, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[4]._x, -1.69047, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[4]._y, 0.3747, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[4]._theta, -5, 0.01);

  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[5]._x, -1.69047, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[5]._y, -0.3747, 0.01);
  EXPECT_NEAR(smac_planner::NodeSE2::motion_table.projections[5]._theta, 5, 0.01);

  nav2_costmap_2d::Costmap2D costmapA(100, 100, 0.05, 0.0, 0.0, 0);
  smac_planner::GridCollisionChecker checker(&costmapA);
  smac_planner::NodeSE2 * node = new smac_planner::NodeSE2(49);
  std::function<bool(const unsigned int &, smac_planner::NodeSE2 * &)> neighborGetter =
    [&, this](const unsigned int & index, smac_planner::NodeSE2 * & neighbor_rtn) -> bool
    {
      // because we don't return a real object
      return false;
    };

  smac_planner::NodeSE2::NodeVector neighbors;
  smac_planner::NodeSE2::getNeighbors(node, neighborGetter, checker, false, neighbors);
  delete node;

  // should be empty since totally invalid
  EXPECT_EQ(neighbors.size(), 0u);
}
