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
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_theta_star_planner/theta_star_planner.hpp"

class init_rclcpp
{
public:
  init_rclcpp() {rclcpp::init(0, nullptr);}
  ~init_rclcpp() {rclcpp::shutdown();}
};

/// class created to access the protected members of the ThetaStar class
/// u is used as shorthand for use
class test_theta_star : public theta_star::ThetaStar
{
public:
  int getSizeOfNodePosition()
  {
    return static_cast<int>(node_position_.size());
  }

  bool ulosCheck(const int & x0, const int & y0, const int & x1, const int & y1, double & sl_cost)
  {
    return losCheck(x0, y0, x1, y1, sl_cost);
  }

  bool uwithinLimits(const int & cx, const int & cy) {return withinLimits(cx, cy);}

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

  bool runAlgo(std::vector<coordsW> & path)
  {
    if (!isUnsafeToPlan()) {
      return generatePath(path);
    }
    return false;
  }
};

init_rclcpp node;

// Tests meant to test the algorithm itself and its helper functions
TEST(ThetaStarTest, test_theta_star) {
  auto planner_ = std::make_unique<test_theta_star>();
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
  rclcpp_lifecycle::LifecycleNode::SharedPtr life_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  auto planner_2d = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  planner_2d->configure(life_node, "test", nullptr, costmap_ros);
  planner_2d->activate();

  nav_msgs::msg::Path path = planner_2d->createPlan(start, goal);
  EXPECT_GT(static_cast<int>(path.poses.size()), 0);

  // test if the goal is unsafe
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      costmap_ros->getCostmap()->setCost(i, j, 254);
    }
  }
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  path = planner_2d->createPlan(start, goal);
  EXPECT_EQ(static_cast<int>(path.poses.size()), 0);

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  life_node.reset();
  costmap_ros.reset();
}

TEST(ThetaStarPlanner, test_theta_star_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr life_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("ThetaStarPlannerTest");

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
      rclcpp::Parameter("test.allow_unknown", false)});

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

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);
}
