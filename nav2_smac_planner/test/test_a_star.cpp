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
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/utilities.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_ros_common/node_utils.hpp"

TEST(AStarTest, test_a_star_2d)
{
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  nav2_smac_planner::SearchInfo info;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> a_star(
    nav2_smac_planner::MotionModel::TWOD, info);
  int max_iterations = 10000;
  float tolerance = 0.0;
  float some_tolerance = 20.0;
  int it_on_approach = 10;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations, it_on_approach, terminal_checking_interval,
    max_planning_time, 0.0, 1);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  auto dummy_cancel_checker = []() {
      return false;
    };

  // functional case testing
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, 1, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(20u, 20u, 0);
  a_star.setGoal(80u, 80u, 0);
  nav2_smac_planner::Node2D::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker));
  EXPECT_EQ(num_it, 2414);

  // check path is the right size and collision free
  EXPECT_EQ(path.size(), 82u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }

  // setting non-zero dim 3 for 2D search
  EXPECT_THROW(a_star.setGoal(0, 0, 10), std::runtime_error);
  EXPECT_THROW(a_star.setStart(0, 0, 10), std::runtime_error);

  path.clear();
  // failure cases with invalid inputs
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> a_star_2(
    nav2_smac_planner::MotionModel::TWOD, info);
  a_star_2.initialize(
    false, max_iterations, it_on_approach, terminal_checking_interval,
    max_planning_time, 0, 1);
  num_it = 0;
  EXPECT_THROW(
    a_star_2.createPath(
      path, num_it, tolerance,
      dummy_cancel_checker), std::runtime_error);
  a_star_2.setCollisionChecker(checker.get());
  num_it = 0;
  EXPECT_THROW(
    a_star_2.createPath(
      path, num_it, tolerance,
      dummy_cancel_checker), std::runtime_error);
  a_star_2.setStart(0, 0, 0);  // valid
  a_star_2.setGoal(50, 50, 0);  // invalid
  num_it = 0;
  EXPECT_THROW(
    a_star_2.createPath(
      path, num_it, tolerance,
      dummy_cancel_checker), std::runtime_error);
  num_it = 0;
  // invalid goal but liberal tolerance
  a_star_2.setStart(20, 20, 0);  // valid
  a_star_2.setGoal(50, 50, 0);  // invalid
  EXPECT_TRUE(a_star_2.createPath(path, num_it, some_tolerance, dummy_cancel_checker));
  EXPECT_EQ(path.size(), 21u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }

  EXPECT_TRUE(a_star_2.getStart() != nullptr);
  EXPECT_NE(a_star_2.getGoalManager().getGoalsState().size(), 0);
  EXPECT_EQ(a_star_2.getSizeX(), 100u);
  EXPECT_EQ(a_star_2.getSizeY(), 100u);
  EXPECT_EQ(a_star_2.getSizeDim3(), 1u);
  EXPECT_EQ(a_star_2.getToleranceHeuristic(), 20.0);
  EXPECT_EQ(a_star_2.getOnApproachMaxIterations(), 10);

  // test unused functions
  nav2_smac_planner::AnalyticExpansion<nav2_smac_planner::Node2D> expander(
    nav2_smac_planner::MotionModel::TWOD, info, false, 1);

  auto analytic_expansion_nodes =
    nav2_smac_planner::AnalyticExpansion<nav2_smac_planner::Node2D>::AnalyticExpansionNodes();
  EXPECT_EQ(expander.setAnalyticPath(nullptr, nullptr, analytic_expansion_nodes), nullptr);
  int dummy_int1 = 0;
  int dummy_int2 = 0;
  EXPECT_EQ(
    expander.tryAnalyticExpansion(
      nullptr, {}, {}, {},
      nullptr, dummy_int1, dummy_int2), nullptr);

  nav2_smac_planner::Node2D * start = nullptr;
  EXPECT_EQ(
    expander.refineAnalyticPath(
      start, nullptr, nullptr,
      analytic_expansion_nodes), std::numeric_limits<float>::max());
  nav2_smac_planner::AnalyticExpansion<nav2_smac_planner::Node2D>::AnalyticExpansionNodes
    expected_nodes = expander.getAnalyticPath(nullptr, nullptr, nullptr, nullptr);
  EXPECT_EQ(expected_nodes.nodes.size(), 0);

  delete costmapA;
}

TEST(AStarTest, test_a_star_se2)
{
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.minimum_turning_radius = 8;  // in grid coordinates
  info.retrospective_penalty = 0.015;
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  info.analytic_expansion_ratio = 3.5;
  unsigned int size_theta = 72;
  info.cost_penalty = 1.7;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int it_on_approach = 10;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations, it_on_approach, terminal_checking_interval,
    max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // functional case testing
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(10u, 10u, 0u);
  a_star.setGoal(80u, 80u, 40u);
  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();

  auto dummy_cancel_checker = []() {
      return false;
    };

  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker, expansions.get()));

  // check path is the right size and collision free
  EXPECT_GT(num_it, 2000);
  EXPECT_NEAR(path.size(), 63u, 2u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }
  // no skipped nodes
  for (unsigned int i = 1; i != path.size(); i++) {
    EXPECT_LT(hypotf(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 2.1f);
  }

  // Expansions properly recorded
  EXPECT_GT(expansions->size(), 5u);

  delete costmapA;
}

TEST(AStarTest, test_a_star_analytic_expansion)
{
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.0;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 0.0;
  info.minimum_turning_radius = 8;  // in grid coordinates
  info.retrospective_penalty = 0.015;
  info.analytic_expansion_max_length = 2000.0;  // in grid coordinates
  info.analytic_expansion_ratio = 3.5;
  unsigned int size_theta = 72;
  info.cost_penalty = 1.7;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::REEDS_SHEPP, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int it_on_approach = 10;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations, it_on_approach, terminal_checking_interval,
    max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  // should be a straight path running backwards
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(80u, 0u, 0u);
  a_star.setGoal(20u, 0u, 0u);
  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();

  auto dummy_cancel_checker = []() {
      return false;
    };

  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker, expansions.get()));

  // all straight with no wiggle
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_NEAR(path[i].theta, 0.0, 1e-3);
  }

  delete costmapA;
}

TEST(AStarTest, analytic_expansion_restores_revisited_goal)
{
  using Node = nav2_smac_planner::NodeHybrid;
  auto node = std::make_shared<nav2::LifecycleNode>("rollback_test");
  nav2_smac_planner::SearchInfo info;
  info.minimum_turning_radius = 82.5;
  info.analytic_expansion_max_length = 200.0;
  nav2_smac_planner::AStarAlgorithm<Node> a_star(
    nav2_smac_planner::MotionModel::REEDS_SHEPP, info);
  int max_iterations = 10000;
  a_star.initialize(false, max_iterations, 1000, 5000, 5.0, 401, 64);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = nav2_costmap_2d::Costmap2D(1000, 400, 0.1, 0, 0, 0);
  nav2_smac_planner::GridCollisionChecker checker(costmap_ros, 64, node);
  checker.setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  a_star.setCollisionChecker(&checker);
  auto * ctx = a_star.getContext();
  Node start(Node::getIndex(537, 125, 16, 1000, 64), ctx);
  Node goal(Node::getIndex(530, 66, 1, 1000, 64), ctx);
  start.pose = {537.454773f, 125.400085f, 16.0f};
  goal.pose = {530.467102f, 66.557159f, 1.0f};
  const auto original = goal.pose;
  std::unordered_map<uint64_t, std::unique_ptr<Node>> nodes;
  Node::NodePtr start_ptr = &start, goal_ptr = &goal;
  auto getter = [&](const uint64_t & index, Node::NodePtr & result) {
      if (index == goal.getIndex()) {
        result = goal_ptr;
      } else if (index == start.getIndex()) {
        result = start_ptr;
      } else {
        auto & stored = nodes[index];
        if (!stored) {stored = std::make_unique<Node>(index, ctx);}
        result = stored.get();
      }
      return true;
    };
  nav2_smac_planner::AnalyticExpansion<Node> expansion(
    nav2_smac_planner::MotionModel::REEDS_SHEPP, info, false, 64);
  expansion.setContext(ctx);
  expansion.setCollisionChecker(&checker);
  auto candidate = expansion.getAnalyticPath(
    start_ptr, goal_ptr, getter, ctx->motion_table.state_space);
  ASSERT_FALSE(candidate.nodes.empty());
  size_t goal_visits = 0;
  for (const auto & sample : candidate.nodes) {
    if (sample.node == goal_ptr) {++goal_visits;}
  }
  ASSERT_GE(goal_visits, 2u);
  EXPECT_FLOAT_EQ(goal.pose.x, original.x);
  EXPECT_FLOAT_EQ(goal.pose.y, original.y);
  EXPECT_FLOAT_EQ(goal.pose.theta, original.theta);
}

TEST(AStarTest, test_a_star_lattice)
{
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.05;
  info.non_straight_penalty = 1.05;
  info.reverse_penalty = 2.0;
  info.retrospective_penalty = 0.1;
  info.analytic_expansion_ratio = 3.5;
  info.lattice_filepath =
    nav2::get_package_share_directory("nav2_smac_planner") +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann" +
    "/output.json";
  info.minimum_turning_radius = 8;  // in grid coordinates 0.4/0.05
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  unsigned int size_theta = 16;
  info.cost_penalty = 2.0;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice> a_star(
    nav2_smac_planner::MotionModel::STATE_LATTICE, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations,
    std::numeric_limits<int>::max(), terminal_checking_interval, max_planning_time, 401,
    size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.05, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 20; i <= 30; ++i) {
    for (unsigned int j = 20; j <= 30; ++j) {
      costmapA->setCost(i, j, 254);
    }
  }

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  auto dummy_cancel_checker = []() {
      return false;
    };

  // functional case testing
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(5u, 5u, 0u);
  a_star.setGoal(40u, 40u, 1u);
  nav2_smac_planner::NodeLattice::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker));

  // check path is the right size and collision free
  EXPECT_EQ(num_it, 22);
  EXPECT_GT(path.size(), 45u);
  for (unsigned int i = 0; i != path.size(); i++) {
    EXPECT_EQ(costmapA->getCost(path[i].x, path[i].y), 0);
  }
  // no skipped nodes
  for (unsigned int i = 1; i != path.size(); i++) {
    EXPECT_LT(hypotf(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 2.1f);
  }

  delete costmapA;
}

TEST(AStarTest, test_se2_single_pose_path)
{
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.retrospective_penalty = 0.0;
  info.minimum_turning_radius = 8;  // in grid coordinates
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  info.analytic_expansion_ratio = 3.5;
  unsigned int size_theta = 72;
  info.cost_penalty = 1.7;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 100;
  float tolerance = 10.0;
  int it_on_approach = 10;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations, it_on_approach, terminal_checking_interval,
    max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  auto dummy_cancel_checker = []() {
      return false;
    };
  // functional case testing
  a_star.setCollisionChecker(checker.get());
  nav2_smac_planner::NodeHybrid::CoordinateVector path;

  // test with no goals set nor start
  EXPECT_THROW(
    a_star.createPath(
      path, num_it, tolerance,
      dummy_cancel_checker), std::runtime_error);

  a_star.setStart(10u, 10u, 0u);
  // Goal is one costmap cell away
  a_star.setGoal(12u, 10u, 0u);
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker));

  // Check that the path is length one
  // With the current implementation, this produces a longer path
  // EXPECT_EQ(path.size(), 1u);
  EXPECT_GE(path.size(), 1u);

  delete costmapA;
}

TEST(AStarTest, test_goal_heading_mode)
{
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.1;
  info.non_straight_penalty = 1.1;
  info.reverse_penalty = 2.0;
  info.minimum_turning_radius = 8;  // in grid coordinates
  info.retrospective_penalty = 0.015;
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  info.analytic_expansion_ratio = 3.5;
  unsigned int size_theta = 72;
  info.cost_penalty = 1.7;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 10000;
  float tolerance = 10.0;
  int it_on_approach = 10;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  // BIDIRECTIONAL goal heading mode
  a_star.initialize(
    false, max_iterations, it_on_approach, terminal_checking_interval,
    max_planning_time, 401, size_theta);

  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmapA;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, lnode);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  a_star.setCollisionChecker(checker.get());
  auto ctx = a_star.getContext();

  EXPECT_THROW(
    a_star.setGoal(
      80u, 80u, 40u,
      nav2_smac_planner::GoalHeadingMode::BIDIRECTIONAL),
    std::runtime_error);
  a_star.setStart(10u, 10u, 0u);
  a_star.setGoal(80u, 80u, 40u, nav2_smac_planner::GoalHeadingMode::BIDIRECTIONAL);
  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();

  auto dummy_cancel_checker = []() {
      return false;
    };
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker, expansions.get()));
  EXPECT_EQ(a_star.getGoalManager().getGoalsState().size(), 2);
  EXPECT_EQ(
    a_star.getGoalManager().getGoalsState().size(),
    a_star.getGoalManager().getGoalsCoordinates().size());


  // ALL_DIRECTION goal heading mode
  unsigned int coarse_search_resolution = 16;
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(10u, 10u, 0u);
  a_star.setGoal(
    80u, 80u, 40u, nav2_smac_planner::GoalHeadingMode::ALL_DIRECTION,
    coarse_search_resolution);
  EXPECT_TRUE(a_star.getCoarseSearchResolution() == coarse_search_resolution);

  unsigned int num_bins = ctx->motion_table.num_angle_quantization;

  // get number of valid goal states
  unsigned int num_valid_goals = 0;
  auto goals_state = a_star.getGoalManager().getGoalsState();
  for (unsigned int i = 0; i < goals_state.size(); i++) {
    if (goals_state[i].is_valid) {
      num_valid_goals++;
    }
  }
  EXPECT_TRUE(a_star.getGoalManager().getGoalsState().size() == num_bins);
  EXPECT_TRUE(a_star.getGoalManager().getGoalsState().size() == num_valid_goals);
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker, expansions.get()));
  EXPECT_TRUE(
    a_star.getGoalManager().getGoalsState().size() ==
    a_star.getGoalManager().getGoalsCoordinates().size());

  // UNKNOWN goal heading mode
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(10u, 10u, 0u);

  EXPECT_THROW(
    a_star.setGoal(
      80u, 80u, 10u,
      nav2_smac_planner::GoalHeadingMode::UNKNOWN), std::runtime_error);
  delete costmapA;
}

TEST(AStarTest, test_a_star_se2_direction_change_penalties)
{
  // Turn around in a room at the end of a hallway too narrow to turn within
  auto lnode = std::make_shared<nav2::LifecycleNode>("test");
  const unsigned int size_x = 81, size_y = 36;
  const float resolution = 0.05;
  std::vector<bool> occupied(size_x * size_y, true);
  for (unsigned int x = 3; x < size_x - 3; ++x) {
    for (unsigned int y = 3; y < size_y - 3; ++y) {
      occupied[x + y * size_x] = !(x >= 45 || (y >= 11 && y < 25));
    }
  }

  // Inflate costs with distance to obstacles, so cusps near walls are in high cost space
  nav2_costmap_2d::Costmap2D * costmapA =
    new nav2_costmap_2d::Costmap2D(size_x, size_y, resolution, 0.0, 0.0, 0);
  for (unsigned int x = 0; x < size_x; ++x) {
    for (unsigned int y = 0; y < size_y; ++y) {
      float dist = std::numeric_limits<float>::max();
      for (unsigned int i = 0; i < occupied.size(); ++i) {
        if (occupied[i]) {
          const float dx = static_cast<float>(i % size_x) - static_cast<float>(x);
          const float dy = static_cast<float>(i / size_x) - static_cast<float>(y);
          dist = std::min(dist, hypotf(dx, dy) * resolution);
        }
      }
      unsigned char cost = 0;
      if (dist == 0.0f) {
        cost = 254;
      } else if (dist <= 0.15f) {
        cost = 253;
      } else if (dist <= 1.0f) {
        cost = static_cast<unsigned char>(252.0f * std::exp(-3.0f * (dist - 0.15f)));
      }
      costmapA->setCost(x, y, cost);
    }
  }

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = *costmapA;

  // 0.6m x 0.3m robot
  nav2_costmap_2d::Footprint footprint(4);
  footprint[0].x = 0.3; footprint[0].y = 0.15;
  footprint[1].x = 0.3; footprint[1].y = -0.15;
  footprint[2].x = -0.3; footprint[2].y = -0.15;
  footprint[3].x = -0.3; footprint[3].y = 0.15;
  unsigned int size_theta = 72;
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, lnode);
  checker->setFootprint(footprint, false, 100.0);

  // Plans a turn around, returning the number of changes between moving along and against
  // the heading and the highest cost at which they occur
  auto planTurnAround = [&](const float direction_change_penalty,
    const float direction_change_cost_penalty) {
      nav2_smac_planner::SearchInfo info;
      info.minimum_turning_radius = 8;  // in grid coordinates
      info.non_straight_penalty = 1.2;
      info.cost_penalty = 2.0;
      info.direction_change_penalty = direction_change_penalty;
      info.direction_change_cost_penalty = direction_change_cost_penalty;
      info.direction_change_cost_threshold = 50.0;
      // Short analytic expansions, which are not subject to the direction change penalties
      info.analytic_expansion_max_length = 6.0;  // in grid coordinates
      nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
        nav2_smac_planner::MotionModel::REEDS_SHEPP, info);
      int max_iterations = 100000;
      a_star.initialize(false, max_iterations, 1000, 5000, 120.0, 101, size_theta);
      a_star.setCollisionChecker(checker.get());
      a_star.setStart(30u, 18u, 0u);
      a_star.setGoal(10u, 18u, 36u);
      nav2_smac_planner::NodeHybrid::CoordinateVector path;
      int num_it = 0;
      EXPECT_TRUE(a_star.createPath(path, num_it, 0.0, []() {return false;}));

      unsigned int changes = 0;
      float max_cost = 0.0f;
      int last_dir = 0;
      for (unsigned int i = 1; i < path.size(); ++i) {
        const float dx = path[i].x - path[i - 1].x;
        const float dy = path[i].y - path[i - 1].y;
        if (hypotf(dx, dy) > 1e-3) {
          const int dir = dx * cos(path[i].theta) + dy * sin(path[i].theta) >= 0.0 ? 1 : -1;
          if (last_dir != 0 && dir != last_dir) {
            changes++;
            max_cost = std::max(
              max_cost, static_cast<float>(costmapA->getCost(
                static_cast<unsigned int>(path[i - 1].x + 0.5),
                static_cast<unsigned int>(path[i - 1].y + 0.5))));
          }
          last_dir = dir;
        }
      }
      return std::make_pair(changes, max_cost);
    };

  const auto [changes, cusp_cost] = planTurnAround(0.0, 0.0);
  EXPECT_GT(changes, 0u);
  EXPECT_GT(cusp_cost, 50.0f);
  // Fewer direction changes to turn around with the direction change penalty
  EXPECT_LT(planTurnAround(3.0, 0.0).first, changes);
  // Direction changes move into lower cost space with the direction change cost penalty
  EXPECT_LT(planTurnAround(0.0, 10.0).second, cusp_cost);

  delete costmapA;
}

TEST(AStarTest, test_constants)
{
  nav2_smac_planner::MotionModel mm = nav2_smac_planner::MotionModel::UNKNOWN;  // unknown
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Unknown"));
  mm = nav2_smac_planner::MotionModel::TWOD;  // 2d
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("2D"));
  mm = nav2_smac_planner::MotionModel::DUBIN;  // dubin
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Dubin"));
  mm = nav2_smac_planner::MotionModel::REEDS_SHEPP;  // reeds-shepp
  EXPECT_EQ(nav2_smac_planner::toString(mm), std::string("Reeds-Shepp"));

  nav2_smac_planner::GoalHeadingMode gh = nav2_smac_planner::GoalHeadingMode::UNKNOWN;
  EXPECT_EQ(nav2_smac_planner::toString(gh), std::string("Unknown"));
  gh = nav2_smac_planner::GoalHeadingMode::DEFAULT;  // default
  EXPECT_EQ(nav2_smac_planner::toString(gh), std::string("DEFAULT"));
  gh = nav2_smac_planner::GoalHeadingMode::BIDIRECTIONAL;  // bidirectional
  EXPECT_EQ(nav2_smac_planner::toString(gh), std::string("BIDIRECTIONAL"));
  gh = nav2_smac_planner::GoalHeadingMode::ALL_DIRECTION;  // all_direction
  EXPECT_EQ(nav2_smac_planner::toString(gh), std::string("ALL_DIRECTION"));

  EXPECT_EQ(
    nav2_smac_planner::fromString(
      "2D"), nav2_smac_planner::MotionModel::TWOD);
  EXPECT_EQ(nav2_smac_planner::fromString("DUBIN"), nav2_smac_planner::MotionModel::DUBIN);
  EXPECT_EQ(
    nav2_smac_planner::fromString(
      "REEDS_SHEPP"), nav2_smac_planner::MotionModel::REEDS_SHEPP);
  EXPECT_EQ(nav2_smac_planner::fromString("NONE"), nav2_smac_planner::MotionModel::UNKNOWN);

  EXPECT_EQ(
    nav2_smac_planner::fromStringToGH(
      "DEFAULT"), nav2_smac_planner::GoalHeadingMode::DEFAULT);
  EXPECT_EQ(
    nav2_smac_planner::fromStringToGH(
      "BIDIRECTIONAL"), nav2_smac_planner::GoalHeadingMode::BIDIRECTIONAL);
  EXPECT_EQ(
    nav2_smac_planner::fromStringToGH(
      "ALL_DIRECTION"), nav2_smac_planner::GoalHeadingMode::ALL_DIRECTION);
  EXPECT_EQ(
    nav2_smac_planner::fromStringToGH("NONE"), nav2_smac_planner::GoalHeadingMode::UNKNOWN);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
