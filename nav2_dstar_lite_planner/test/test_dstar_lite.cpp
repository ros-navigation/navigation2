// Copyright (c) 2024 Nav2 Contributors
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
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_dstar_lite_planner/dstar_lite.hpp"
#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"
#include "nav2_dstar_lite_planner/parameter_handler.hpp"

class TestDStarLite : public nav2_dstar_lite_planner::DStarLite
{
public:
  explicit TestDStarLite(nav2_dstar_lite_planner::Parameters * params)
  : DStarLite(params) {}

  // expose protected methods for white-box testing
  using nav2_dstar_lite_planner::DStarLite::isSafe;
  using nav2_dstar_lite_planner::DStarLite::withinBounds;
  using nav2_dstar_lite_planner::DStarLite::initialize;
  using nav2_dstar_lite_planner::DStarLite::computeShortestPath;
  using nav2_dstar_lite_planner::DStarLite::updateVertex;
  using nav2_dstar_lite_planner::DStarLite::calculateKey;
  using nav2_dstar_lite_planner::DStarLite::getHeuristic;

  bool runAlgo(
    std::vector<nav2_dstar_lite_planner::WorldCoord> & path,
    std::function<bool()> cancel_checker = []() {return false;})
  {
    if (!isUnsafeToPlan()) {
      return generatePath(path, cancel_checker);
    }
    return false;
  }

  int getStateCount()
  {
    return next_state_id_;
  }

  bool isInQueue(const nav2_dstar_lite_planner::CellIndex & s)
  {
    auto it = state_lookup_.find(s);
    if (it != state_lookup_.end()) {
      return state_pool_[it->second].in_queue;
    }
    return false;
  }

  double getG(const nav2_dstar_lite_planner::CellIndex & s)
  {
    return getOrCreateState(s).g;
  }

  double getRHS(const nav2_dstar_lite_planner::CellIndex & s)
  {
    return getOrCreateState(s).rhs;
  }
};

// ========== Algorithm Unit Tests ==========

TEST(DStarLiteAlgorithm, test_initialize)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 50;
  planner->size_y_ = 50;

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 5.0;
  start.pose.position.y = 5.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 45.0;
  goal.pose.position.y = 45.0;
  goal.pose.orientation.w = 1.0;

  planner->setStartAndGoal(start, goal);
  planner->initialize();

  nav2_dstar_lite_planner::CellIndex goal_cell = planner->dst_;
  EXPECT_EQ(planner->getRHS(goal_cell), 0.0);
  EXPECT_TRUE(planner->isInQueue(goal_cell));

  nav2_dstar_lite_planner::CellIndex start_cell = planner->src_;
  EXPECT_GT(planner->getRHS(start_cell), 1e100);
  EXPECT_GT(planner->getG(start_cell), 1e100);
}

TEST(DStarLiteAlgorithm, test_isSafe_with_unknown)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  params->allow_unknown = true;

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);

  EXPECT_TRUE(planner->isSafe({5, 5}));

  planner->costmap_->setCost(10, 10, 255);
  EXPECT_TRUE(planner->isSafe({10, 10}));

  planner->costmap_->setCost(10, 11, 254);
  EXPECT_FALSE(planner->isSafe({10, 11}));

  params->allow_unknown = false;
  EXPECT_FALSE(planner->isSafe({10, 10}));
}

TEST(DStarLiteAlgorithm, test_withinBounds)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  auto params = param_handler->getParams();

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->size_x_ = 50;
  planner->size_y_ = 50;

  EXPECT_TRUE(planner->withinBounds({0, 0}));
  EXPECT_TRUE(planner->withinBounds({49, 49}));
  EXPECT_TRUE(planner->withinBounds({25, 25}));
  EXPECT_FALSE(planner->withinBounds({50, 25}));
  EXPECT_FALSE(planner->withinBounds({25, 50}));
  EXPECT_FALSE(planner->withinBounds({-1, 0}));
  EXPECT_FALSE(planner->withinBounds({0, -1}));
}

TEST(DStarLiteAlgorithm, test_simple_path)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 50;
  planner->size_y_ = 50;

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 5.0;
  start.pose.position.y = 5.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 45.0;
  goal.pose.position.y = 45.0;
  goal.pose.orientation.w = 1.0;

  planner->setStartAndGoal(start, goal);
  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  EXPECT_TRUE(planner->runAlgo(path));
  EXPECT_GT(path.size(), 0u);

  double dx = path.back().x - 45.0;
  double dy = path.back().y - 45.0;
  EXPECT_LT(std::hypot(dx, dy), 1.0);
}

TEST(DStarLiteAlgorithm, test_path_around_obstacle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 50;
  planner->size_y_ = 50;

  // Small obstacle block at (15, 23) to (15, 27) — path must go around
  for (int y = 23; y <= 27; y++) {
    planner->costmap_->setCost(15, y, 254);
  }

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 5.0;
  start.pose.position.y = 25.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 25.0;
  goal.pose.position.y = 25.0;
  goal.pose.orientation.w = 1.0;

  planner->setStartAndGoal(start, goal);
  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  EXPECT_TRUE(planner->runAlgo(path));
  EXPECT_GT(path.size(), 0u);

  EXPECT_LT(std::hypot(path.back().x - 25.0, path.back().y - 25.0), 1.0);
}

TEST(DStarLiteAlgorithm, test_no_path)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 50;
  planner->size_y_ = 50;

  for (int x = 0; x < 50; x++) {
    for (int y = 0; y < 50; y++) {
      planner->costmap_->setCost(x, y, 254);
    }
  }

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 5.0;
  start.pose.position.y = 5.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 45.0;
  goal.pose.position.y = 45.0;
  goal.pose.orientation.w = 1.0;

  planner->setStartAndGoal(start, goal);
  std::vector<nav2_dstar_lite_planner::WorldCoord> path;
  EXPECT_FALSE(planner->runAlgo(path));
  EXPECT_EQ(path.size(), 0u);
}

TEST(DStarLiteAlgorithm, test_incremental_replan)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLiteTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_dstar_lite_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  auto planner = std::make_unique<TestDStarLite>(params);
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 50;
  planner->size_y_ = 50;

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 5.0;
  start.pose.position.y = 5.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 45.0;
  goal.pose.position.y = 45.0;
  goal.pose.orientation.w = 1.0;

  planner->clearStart();
  planner->setStartAndGoal(start, goal);
  std::vector<nav2_dstar_lite_planner::WorldCoord> path1;
  EXPECT_TRUE(planner->runAlgo(path1));
  EXPECT_GT(path1.size(), 0u);

  planner->costmap_->setCost(10, 10, 254);
  planner->forceFullReplan();

  std::vector<nav2_dstar_lite_planner::WorldCoord> path2;
  EXPECT_TRUE(planner->runAlgo(path2));
  EXPECT_GT(path2.size(), 0u);
}

// ========== Plugin Integration Tests ==========

TEST(DStarLitePlanner, test_lifecycle)
{
  auto life_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLitePlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped start, goal, viapoint;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  viapoint = start;

  auto planner_2d = std::make_unique<nav2_dstar_lite_planner::DStarLitePlanner>();
  planner_2d->configure(life_node, "test", nullptr, costmap_ros);
  planner_2d->activate();

  nav_msgs::msg::Path path = planner_2d->createPlan(start, goal);
  EXPECT_GT(path.poses.size(), 0u);

  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      costmap_ros->getCostmap()->setCost(i, j, 254);
    }
  }
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;

  EXPECT_THROW(
    planner_2d->createPlan(start, goal),
    nav2_core::PlannerException);

  planner_2d->deactivate();
  planner_2d->cleanup();
  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  life_node.reset();
  costmap_ros.reset();
}

TEST(DStarLitePlanner, test_reconfigure)
{
  auto life_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DStarLitePlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<nav2_dstar_lite_planner::DStarLitePlanner>();
  planner->configure(life_node, "test", nullptr, costmap_ros);
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    life_node->get_node_base_interface(), life_node->get_node_topics_interface(),
    life_node->get_node_graph_interface(),
    life_node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.max_iterations", 50000),
      rclcpp::Parameter("test.hysteresis_factor", 1.1),
      rclcpp::Parameter("test.use_final_approach_orientation", false),
      rclcpp::Parameter("test.terminal_checking_interval", 100)});

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(), results);

  EXPECT_EQ(life_node->get_parameter("test.allow_unknown").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.max_iterations").as_int(), 50000);
  EXPECT_EQ(life_node->get_parameter("test.hysteresis_factor").as_double(), 1.1);
  EXPECT_EQ(life_node->get_parameter("test.use_final_approach_orientation").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.terminal_checking_interval").as_int(), 100);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.hysteresis_factor", 0.5)});
  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(), results);
  EXPECT_EQ(life_node->get_parameter("test.hysteresis_factor").as_double(), 1.1);

  planner->deactivate();
  planner->cleanup();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
