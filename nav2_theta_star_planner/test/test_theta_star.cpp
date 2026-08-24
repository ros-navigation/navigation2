//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/parameter_handler.hpp"

/// class created to access the protected members of the ThetaStar class
/// u is used as shorthand for use
class test_theta_star : public nav2_theta_star_planner::ThetaStar
{
public:
  explicit test_theta_star(nav2_theta_star_planner::Parameters * params)
  : ThetaStar(params) {}
  int getSizeOfNodePosition()
  {
    return static_cast<int>(node_position_.size());
  }

  bool ulosCheck(const int & x0, const int & y0, const int & x1, const int & y1, double & sl_cost)
  {
    return losCheck(x0, y0, x1, y1, sl_cost);
  }

  bool uwithinLimits(const int & cx, const int & cy) {return withinLimits(cx, cy);}

  double ugetTraversalCost(const int & cx, const int & cy) {return getTraversalCost(cx, cy);}

  bool uisGoal(const tree_node & this_node) {return isGoal(this_node);}

  void uinitializePosn(int size_inc = 0)
  {
    node_position_.reserve(size_x_ * size_y_); initializePosn(size_inc);
  }

  void uaddIndex(const int & cx, const int & cy) {addIndex(cx, cy, &nodes_data_[0]);}

  tree_node * ugetIndex(const int & cx, const int & cy) {return getIndex(cx, cy);}

  tree_node * test_getIndex() {return &nodes_data_[0];}

  void uaddToNodesData(const int & id) {addToNodesData(id);}

  void uresetContainers() {nodes_data_.clear(); resetContainers();}

  bool runAlgo(
    std::vector<coordsW> & path,
    std::function<bool()> cancel_checker = [] () {return false;})
  {
    if (!isUnsafeToPlan()) {
      return generatePath(path, cancel_checker);
    }
    return false;
  }
};

// Tests meant to test the algorithm itself and its helper functions
TEST(ThetaStarTest, test_theta_star) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);
  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      planner_->costmap_->setCost(i, j, 253);
    }
  }
  coordsM s = {5, 5}, g = {18, 18};
  int size_x = 20, size_y = 20;
  planner_->size_x_ = size_x;
  planner_->size_y_ = size_y;
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = s.x;
  start.pose.position.y = s.y;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = g.x;
  goal.pose.position.y = g.y;
  goal.pose.orientation.w = 1.0;
  /// Check if the setStartAndGoal function works properly
  planner_->setStartAndGoal(start, goal);
  EXPECT_TRUE(planner_->src_.x == s.x && planner_->src_.y == s.y);
  EXPECT_TRUE(planner_->dst_.x == g.x && planner_->dst_.y == g.y);
  /// Check if the initializePosn function works properly
  planner_->uinitializePosn(size_x * size_y);
  EXPECT_EQ(planner_->getSizeOfNodePosition(), (size_x * size_y));

  /// Check if the withinLimits function works properly
  EXPECT_TRUE(planner_->uwithinLimits(18, 18));
  EXPECT_FALSE(planner_->uwithinLimits(120, 140));

  tree_node n = {g.x, g.y, 120, 0, NULL, false, 20};
  n.parent_id = &n;
  /// Check if the isGoal function works properly
  EXPECT_TRUE(planner_->uisGoal(n));           // both (x,y) are the goal coordinates
  n.x = 25;
  EXPECT_FALSE(planner_->uisGoal(n));          // only y coordinate matches with that of goal
  n.x = g.x;
  n.y = 20;
  EXPECT_FALSE(planner_->uisGoal(n));          // only x coordinate matches with that of goal
  n.x = 30;
  EXPECT_FALSE(planner_->uisGoal(n));          // both (x, y) are different from the goal coordinate

  /// Check if the isSafe functions work properly
  EXPECT_TRUE(planner_->isSafe(5, 5));         // cost at this point is 0
  EXPECT_FALSE(planner_->isSafe(10, 10));      // cost at this point is 253 (>LETHAL_COST)

  /// Check if the functions addIndex & getIndex work properly
  coordsM c = {18, 18};
  planner_->uaddToNodesData(0);
  planner_->uaddIndex(c.x, c.y);
  tree_node * c_node = planner_->ugetIndex(c.x, c.y);
  EXPECT_EQ(c_node, planner_->test_getIndex());

  double sl_cost = 0.0;
  /// Checking for the case where the losCheck should return the value as true
  EXPECT_TRUE(planner_->ulosCheck(2, 2, 7, 20, sl_cost));
  /// and as false
  EXPECT_FALSE(planner_->ulosCheck(2, 2, 18, 18, sl_cost));

  planner_->uresetContainers();
  std::vector<coordsW> path;
  /// Check if the planner returns a path for the case where a path exists
  EXPECT_TRUE(planner_->runAlgo(path));
  EXPECT_GT(static_cast<int>(path.size()), 0);
  /// and where it doesn't exist
  path.clear();
  planner_->src_ = {10, 10};
  EXPECT_FALSE(planner_->runAlgo(path));
  EXPECT_EQ(static_cast<int>(path.size()), 0);
}

// Smoke tests meant to detect issues arising from the plugin part rather than the algorithm
TEST(ThetaStarPlanner, test_theta_star_planner) {
  nav2::LifecycleNode::SharedPtr life_node =
    std::make_shared<nav2::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped start, goal, viapoint;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  viapoint = start;
  auto planner_2d = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  planner_2d->configure(life_node, "test", nullptr, costmap_ros);
  planner_2d->activate();

  auto dummy_cancel_checker = []() {
      return false;
    };

  std::vector<geometry_msgs::msg::PoseStamped> viapoints{viapoint};
  nav_msgs::msg::Path path = planner_2d->createPlan(
    start, goal, viapoints, dummy_cancel_checker);
  EXPECT_GT(static_cast<int>(path.poses.size()), 0);

  // test if the goal is unsafe
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      costmap_ros->getCostmap()->setCost(i, j, 254);
    }
  }
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;

  EXPECT_THROW(planner_2d->createPlan(start, goal, viapoints, dummy_cancel_checker),
    nav2_core::GoalOccupied);

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  life_node.reset();
  costmap_ros.reset();
}

TEST(ThetaStarPlanner, test_theta_star_reconfigure)
{
  nav2::LifecycleNode::SharedPtr life_node =
    std::make_shared<nav2::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(life_node, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    life_node->get_node_base_interface(), life_node->get_node_topics_interface(),
    life_node->get_node_graph_interface(),
    life_node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.how_many_corners", 8),
      rclcpp::Parameter("test.w_euc_cost", 1.0),
      rclcpp::Parameter("test.w_traversal_cost", 2.0),
      rclcpp::Parameter("test.use_final_approach_orientation", false),
      rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.terminal_checking_interval", 100)});

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);

  EXPECT_EQ(life_node->get_parameter("test.how_many_corners").as_int(), 8);
  EXPECT_EQ(
    life_node->get_parameter("test.w_euc_cost").as_double(),
    1.0);
  EXPECT_EQ(life_node->get_parameter("test.w_traversal_cost").as_double(), 2.0);
  EXPECT_EQ(life_node->get_parameter("test.use_final_approach_orientation").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.allow_unknown").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.terminal_checking_interval").as_int(), 100);

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);

  // Try setting invalid value for how_many_corners
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.how_many_corners", 5)});
  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);
  EXPECT_EQ(life_node->get_parameter("test.how_many_corners").as_int(), 8);

  // Try setting invalid value for w_euc_cost
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.w_euc_cost", -1.0)});
  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);
  EXPECT_EQ(life_node->get_parameter("test.w_euc_cost").as_double(), 1.0);
}

/// The traversal cost is charged per unit distance, so two straight lines of equal metric length
/// accrue equal cost whatever their bearing. Charging once per Bresenham step regardless of
/// whether the step is axial or diagonal makes a 45-degree line about 29% cheaper per metre.
TEST(ThetaStarTest, test_los_cost_is_direction_independent) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarDirectionTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  /// A uniform costmap: every cell has the same cost density, so the cost of a line is exactly
  /// proportional to its length and any residual bearing dependence is the defect under test.
  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(120, 120, 1.0, 0.0, 0.0, 100);
  params->w_traversal_cost = 2.0;

  const int line_length = 100;
  double axial_cost = 0.0, diagonal_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 5 + line_length, 5, axial_cost));
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 5 + line_length, 5 + line_length, diagonal_cost));

  EXPECT_NEAR(
    axial_cost / line_length,
    diagonal_cost / std::hypot(line_length, line_length),
    1e-9);

  /// Axial and 45-degree are the two bearings at which the Bresenham staircase happens to be the
  /// same length as the line, so an off-axis bearing is needed to detect a charge that follows the
  /// staircase rather than the line.
  double oblique_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 5 + line_length, 5 + line_length / 2, oblique_cost));
  EXPECT_NEAR(
    axial_cost / line_length,
    oblique_cost / std::hypot(line_length, line_length / 2),
    1e-9);

  delete planner_->costmap_;
}

/// Each expansion step is charged by its metric length, so on a uniform costmap the planner
/// reaches an off-axis goal by a straight path rather than one bowed towards the grid diagonals.
TEST(ThetaStarTest, test_path_does_not_bow_on_uniform_costmap) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarBowTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(200, 200, 1.0, 0.0, 0.0, 100);
  params->w_euc_cost = 1.0;
  params->w_traversal_cost = 2.0;
  params->w_heuristic_cost = 1.0;
  params->how_many_corners = 8;

  /// An off-grid bearing, so neither the axial nor the diagonal step is favoured outright.
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 10;
  start.pose.position.y = 10;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 190;
  goal.pose.position.y = 120;
  goal.pose.orientation.w = 1.0;
  planner_->setStartAndGoal(start, goal);

  std::vector<coordsW> path;
  ASSERT_TRUE(planner_->runAlgo(path));
  ASSERT_GE(static_cast<int>(path.size()), 2);

  double length = 0.0;
  for (size_t i = 1; i < path.size(); i++) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  const double chord = std::hypot(
    path.back().x - path.front().x, path.back().y - path.front().y);
  /// A path bowed towards the diagonals is measurably longer than its own chord.
  EXPECT_LT(length / chord, 1.001);

  delete planner_->costmap_;
}

/// w_euc_cost is the only thing charging for path length, so a non-positive value makes every
/// path through free space cost the same and the planner returns arbitrary ones. The dynamic
/// reconfigure path already rejects non-positive doubles; the load path must agree.
/// w_heuristic_cost is derived from w_euc_cost and has to track it, including on reconfigure,
/// or the heuristic can exceed the true remaining cost and the search stops being admissible.
TEST(ThetaStarPlanner, test_w_euc_cost_validation) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarEucValidationNode");
  auto plugin_name = std::string("test");
  node->declare_parameter(plugin_name + ".w_euc_cost", rclcpp::ParameterValue(0.0));

  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  /// A non-positive w_euc_cost loaded from config is overridden rather than accepted.
  EXPECT_GT(params->w_euc_cost, 0.0);
  EXPECT_GT(params->w_heuristic_cost, 0.0);

  /// and the heuristic weight tracks w_euc_cost when it changes at runtime.
  auto result = node->set_parameters(
    {rclcpp::Parameter(plugin_name + ".w_euc_cost", 0.5)});
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result[0].successful);
  EXPECT_DOUBLE_EQ(params->w_euc_cost, 0.5);
  EXPECT_DOUBLE_EQ(params->w_heuristic_cost, 0.5);

  /// A non-positive value is still refused on reconfigure.
  auto bad = node->set_parameters(
    {rclcpp::Parameter(plugin_name + ".w_euc_cost", 0.0)});
  ASSERT_EQ(bad.size(), 1u);
  EXPECT_FALSE(bad[0].successful);
  EXPECT_DOUBLE_EQ(params->w_euc_cost, 0.5);

  /// A non-finite value passes a bare "<= 0.0" test, so it must be refused explicitly.
  auto nan_result = node->set_parameters(
    {rclcpp::Parameter(plugin_name + ".w_euc_cost", std::nan(""))});
  ASSERT_EQ(nan_result.size(), 1u);
  EXPECT_FALSE(nan_result[0].successful);
  auto inf_result = node->set_parameters(
    {rclcpp::Parameter(
      plugin_name + ".w_euc_cost", std::numeric_limits<double>::infinity())});
  ASSERT_EQ(inf_result.size(), 1u);
  EXPECT_FALSE(inf_result[0].successful);
  EXPECT_DOUBLE_EQ(params->w_euc_cost, 0.5);
}

/// w_traversal_cost is validated on reconfigure but was not at load. With the free-space floor
/// gone a negative weight makes the per-unit charge negative on high-cost cells, which is a
/// negative edge weight and invalidates the search ordering.
TEST(ThetaStarPlanner, test_w_traversal_cost_validation) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarTraversalValidationNode");
  auto plugin_name = std::string("test");
  node->declare_parameter(plugin_name + ".w_traversal_cost", rclcpp::ParameterValue(-2.0));

  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  EXPECT_GT(params->w_traversal_cost, 0.0);
}

/// Non-finite weights pass a bare "<= 0.0" test, so the load path must reject them explicitly,
/// just as the reconfigure path does.
TEST(ThetaStarPlanner, test_non_finite_weight_at_load) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarNonFiniteLoadNode");
  auto plugin_name = std::string("test");
  node->declare_parameter(plugin_name + ".w_euc_cost", rclcpp::ParameterValue(std::nan("")));
  node->declare_parameter(
    plugin_name + ".w_traversal_cost",
    rclcpp::ParameterValue(std::numeric_limits<double>::infinity()));

  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();

  EXPECT_TRUE(std::isfinite(params->w_euc_cost));
  EXPECT_GT(params->w_euc_cost, 0.0);
  EXPECT_TRUE(std::isfinite(params->w_traversal_cost));
  EXPECT_GT(params->w_traversal_cost, 0.0);
  EXPECT_TRUE(std::isfinite(params->w_heuristic_cost));
}

/// Free space carries no traversal cost, so the traversal term contributes nothing to a line
/// that crosses only free cells, and the cost of such a line is charged solely by w_euc_cost.
/// The safety cutoff also becomes consistent: both isSafe overloads then admit exactly the same
/// set of cells, where the 26 + 0.9 remap made the line-of-sight one stop a cost level earlier.
TEST(ThetaStarTest, test_free_space_carries_no_traversal_cost) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarFreeSpaceTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(120, 120, 1.0, 0.0, 0.0, 0);
  params->w_traversal_cost = 2.0;

  double sl_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 105, 5, sl_cost));
  EXPECT_DOUBLE_EQ(sl_cost, 0.0);

  /// A uniform non-free cost is still charged at its analytic density per unit distance.
  for (int i = 0; i < 120; i++) {
    for (int j = 0; j < 120; j++) {
      planner_->costmap_->setCost(i, j, 126);
    }
  }
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 105, 5, sl_cost));
  EXPECT_NEAR(sl_cost / 100.0, 2.0 * (126.0 / 252.0) * (126.0 / 252.0), 1e-9);

  /// The highest non-obstacle cost is admitted by both overloads, not just the plain one.
  planner_->costmap_->setCost(50, 5, MAX_NON_OBSTACLE_COST);
  EXPECT_TRUE(planner_->isSafe(50, 5));
  EXPECT_TRUE(planner_->ulosCheck(5, 5, 105, 5, sl_cost));

  delete planner_->costmap_;
}

/// An unknown cell is charged as near-obstacle by both cost sites. Reading the raw costmap value
/// at one site and the clamped value at the other made the same cell cost different amounts
/// depending on whether it was reached by an expansion step or crossed by a line-of-sight check.
TEST(ThetaStarTest, test_unknown_cost_agrees_between_cost_sites) {
  auto node = std::make_shared<nav2::LifecycleNode>("ThetaStarUnknownTestNode");
  auto plugin_name = std::string("test");
  auto param_handler = std::make_unique<nav2_theta_star_planner::ParameterHandler>(
    node, plugin_name, node->get_logger());
  param_handler->activate();
  auto params = param_handler->getParams();
  auto planner_ = std::make_unique<test_theta_star>(params);

  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(120, 120, 1.0, 0.0, 0.0, UNKNOWN_COST);
  params->w_traversal_cost = 2.0;
  params->allow_unknown = true;

  /// The per-cell charge the line-of-sight check makes, recovered per unit distance.
  double sl_cost = 0.0;
  ASSERT_TRUE(planner_->ulosCheck(5, 5, 105, 5, sl_cost));
  const double los_charge = sl_cost / 100.0;

  /// and the charge an expansion step makes for the same cell.
  const double step_charge = planner_->ugetTraversalCost(50, 50);

  EXPECT_NEAR(los_charge, step_charge, 1e-9);
  /// both being the near-obstacle value, not the raw UNKNOWN_COST of 255
  const double expected = 2.0 *
    ((OCCUPIED_COST - 1) / static_cast<double>(MAX_NON_OBSTACLE_COST)) *
    ((OCCUPIED_COST - 1) / static_cast<double>(MAX_NON_OBSTACLE_COST));
  EXPECT_NEAR(step_charge, expected, 1e-9);

  delete planner_->costmap_;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
