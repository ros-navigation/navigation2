// Copyright (c) 2021 Joshua Wallace
// Copyright (c) 2021 Samsung Research America
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

#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <limits>
#include "nav2_smac_planner/node_lattice.hpp"
#include "gtest/gtest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/lifecycle_node.hpp"

using json = nlohmann::json;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NodeLatticeTest, parser_test)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";
  std::ifstream myJsonFile(filePath);

  ASSERT_TRUE(myJsonFile.is_open());

  json j;
  myJsonFile >> j;

  nav2_smac_planner::LatticeMetadata metaData;
  nav2_smac_planner::MotionPrimitive myPrimitive;
  nav2_smac_planner::MotionPose pose;

  json jsonMetaData = j["lattice_metadata"];
  json jsonPrimatives = j["primitives"];
  json jsonPose = jsonPrimatives[0]["poses"][0];

  nav2_smac_planner::fromJsonToMetaData(jsonMetaData, metaData);

  // Checks for parsing meta data
  EXPECT_NEAR(metaData.min_turning_radius, 0.5, 0.001);
  EXPECT_NEAR(metaData.grid_resolution, 0.05, 0.001);
  EXPECT_NEAR(metaData.number_of_headings, 16, 0.01);
  EXPECT_NEAR(metaData.heading_angles[0], 0.0, 0.01);
  EXPECT_EQ(metaData.number_of_trajectories, 80u);
  EXPECT_EQ(metaData.motion_model, std::string("ackermann"));

  std::vector<nav2_smac_planner::MotionPrimitive> myPrimitives;
  for (unsigned int i = 0; i < jsonPrimatives.size(); ++i) {
    nav2_smac_planner::MotionPrimitive newPrimative;
    nav2_smac_planner::fromJsonToMotionPrimitive(jsonPrimatives[i], newPrimative);
    myPrimitives.push_back(newPrimative);
  }

  // Checks for parsing primitives
  EXPECT_EQ(myPrimitives.size(), 80u);
  EXPECT_NEAR(myPrimitives[0].trajectory_id, 0, 0.01);
  EXPECT_NEAR(myPrimitives[0].start_angle, 0.0, 0.01);
  EXPECT_NEAR(myPrimitives[0].end_angle, 13, 0.01);
  EXPECT_NEAR(myPrimitives[0].turning_radius, 0.5259, 0.01);
  EXPECT_NEAR(myPrimitives[0].trajectory_length, 0.64856, 0.01);
  EXPECT_NEAR(myPrimitives[0].arc_length, 0.58225, 0.01);
  EXPECT_NEAR(myPrimitives[0].straight_length, 0.06631, 0.01);

  EXPECT_NEAR(myPrimitives[0].poses[0]._x, 0.04981, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[0]._y, -0.00236, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[0]._theta, 6.1883, 0.01);

  EXPECT_NEAR(myPrimitives[0].poses[1]._x, 0.09917, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[1]._y, -0.00944, 0.01);
  EXPECT_NEAR(myPrimitives[0].poses[1]._theta, 6.09345, 0.015);
}

TEST(NodeLatticeTest, test_node_lattice_neighbors_and_parsing)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.0;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice aNode(0);
  unsigned int direction_change_index = 0;
  aNode.setPose(nav2_smac_planner::NodeHybrid::Coordinates(0, 0, 0));
  nav2_smac_planner::MotionPrimitivePtrs projections =
    nav2_smac_planner::NodeLattice::motion_table.getMotionPrimitives(
    &aNode,
    direction_change_index);

  EXPECT_NEAR(projections[0]->poses.back()._x, 0.5, 0.01);
  EXPECT_NEAR(projections[0]->poses.back()._y, -0.35, 0.01);
  EXPECT_NEAR(projections[0]->poses.back()._theta, 5.176, 0.01);

  EXPECT_NEAR(
    nav2_smac_planner::NodeLattice::motion_table.getLatticeMetadata(
      filePath)
    .grid_resolution,
    0.05, 0.005);
}

TEST(NodeLatticeTest, test_node_lattice_conversions)
{
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.0;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice aNode(0);
  aNode.setPose(nav2_smac_planner::NodeHybrid::Coordinates(0, 0, 0));

  EXPECT_NEAR(aNode.motion_table.getAngleFromBin(0u), 0.0, 0.005);
  EXPECT_NEAR(aNode.motion_table.getAngleFromBin(1u), 0.46364, 0.005);
  EXPECT_NEAR(aNode.motion_table.getAngleFromBin(2u), 0.78539, 0.005);

  EXPECT_EQ(aNode.motion_table.getClosestAngularBin(0.0), 0u);
  EXPECT_EQ(aNode.motion_table.getClosestAngularBin(0.5), 1u);
  EXPECT_EQ(aNode.motion_table.getClosestAngularBin(1.5), 4u);
}

TEST(NodeLatticeTest, test_node_lattice)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.1;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  // Check defaults
  nav2_smac_planner::NodeLattice aNode(0);
  nav2_smac_planner::NodeLattice testA(49);
  EXPECT_EQ(testA.getIndex(), 49u);
  EXPECT_EQ(testA.getAccumulatedCost(), std::numeric_limits<float>::max());
  EXPECT_TRUE(std::isnan(testA.getCost()));
  EXPECT_EQ(testA.getMotionPrimitive(), nullptr);

  // Test visited state / reset
  EXPECT_EQ(testA.wasVisited(), false);
  testA.visited();
  EXPECT_EQ(testA.wasVisited(), true);
  testA.reset();
  EXPECT_EQ(testA.wasVisited(), false);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    10, 10, 0.05, 0.0, 0.0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, 72, node);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // test node valid and cost
  testA.pose.x = 5;
  testA.pose.y = 5;
  testA.pose.theta = 0;
  EXPECT_EQ(testA.isNodeValid(true, checker.get()), true);
  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);
  EXPECT_EQ(testA.getCost(), 0.0f);

  // check collision checking
  EXPECT_EQ(testA.isNodeValid(false, checker.get()), true);

  // check operator== works on index
  nav2_smac_planner::NodeLattice testC(49);
  EXPECT_TRUE(testA == testC);

  // check accumulated costs are set
  testC.setAccumulatedCost(100);
  EXPECT_EQ(testC.getAccumulatedCost(), 100.0f);

  // check set pose and pose
  testC.setPose(nav2_smac_planner::NodeLattice::Coordinates(10.0, 5.0, 4));
  EXPECT_EQ(testC.pose.x, 10.0);
  EXPECT_EQ(testC.pose.y, 5.0);
  EXPECT_EQ(testC.pose.theta, 4);

  delete costmapA;
}

TEST(NodeLatticeTest, test_get_neighbors)
{
  auto lnode = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 1.1;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.analytic_expansion_ratio = 1;
  info.retrospective_penalty = 0.0;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice node(49);

  nav2_costmap_2d::Costmap2D * costmapA = new nav2_costmap_2d::Costmap2D(
    10, 10, 0.05, 0.0, 0.0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, 72, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  std::function<bool(const uint64_t &,
    nav2_smac_planner::NodeLattice * &)> neighborGetter =
    [&, this](const uint64_t & index,
    nav2_smac_planner::NodeLattice * & neighbor_rtn) -> bool
    {
      // because we don't return a real object
      return false;
    };

  nav2_smac_planner::NodeLattice::NodeVector neighbors;
  node.getNeighbors(neighborGetter, checker.get(), false, neighbors);
  // should be empty since totally invalid
  EXPECT_EQ(neighbors.size(), 0u);

  delete costmapA;
}

TEST(NodeLatticeTest, test_node_lattice_custom_footprint)
{
  auto lnode = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
  std::string filePath =
    pkg_share_dir +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";

  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 0.5;
  info.non_straight_penalty = 1;
  info.change_penalty = 1;
  info.reverse_penalty = 1;
  info.cost_penalty = 1;
  info.retrospective_penalty = 0.1;
  info.analytic_expansion_ratio = 1;
  info.lattice_filepath = filePath;
  info.cache_obstacle_heuristic = true;
  info.allow_reverse_expansion = true;

  unsigned int x = 100;
  unsigned int y = 100;
  unsigned int angle_quantization = 16;

  nav2_smac_planner::NodeLattice::initMotionModel(
    nav2_smac_planner::MotionModel::STATE_LATTICE, x, y, angle_quantization, info);

  nav2_smac_planner::NodeLattice node(49);

  nav2_costmap_2d::Costmap2D * costmap = new nav2_costmap_2d::Costmap2D(
    40, 40, 0.05, 0.0, 0.0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmapi = costmap_ros->getCostmap();
  *costmapi = *costmap;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, 72, lnode);

  // Make some custom asymmetrical footprint
  nav2_costmap_2d::Footprint footprint;
  geometry_msgs::msg::Point p;
  p.x = -0.1;
  p.y = -0.15;
  footprint.push_back(p);
  p.x = 0.35;
  p.y = -0.15;
  footprint.push_back(p);
  p.x = 0.35;
  p.y = 0.22;
  footprint.push_back(p);
  p.x = -0.1;
  p.y = 0.22;
  footprint.push_back(p);
  checker->setFootprint(footprint, false, 0.0);

  // Setting initial robot pose to (1.0, 1.0, 0.0)
  node.pose.x = 20;
  node.pose.y = 20;
  node.pose.theta = 0;

  // initialize direction change index
  unsigned int direction_change_index = 0;
  // Test that the node is valid though all motion primitives poses for custom footprint
  nav2_smac_planner::MotionPrimitivePtrs motion_primitives =
    nav2_smac_planner::NodeLattice::motion_table.getMotionPrimitives(&node, direction_change_index);
  EXPECT_GT(motion_primitives.size(), 0u);
  for (unsigned int i = 0; i < motion_primitives.size(); i++) {
    EXPECT_EQ(node.isNodeValid(true, checker.get(), motion_primitives[i], false), true);
    EXPECT_EQ(node.isNodeValid(true, checker.get(), motion_primitives[i], true), true);
  }

  delete costmap;
}
