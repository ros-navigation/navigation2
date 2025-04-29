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

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"

// Simple wrapper to be able to call a private member
class LatticeWrap : public nav2_smac_planner::SmacPlannerLattice
{
public:
  void callDynamicParams(std::vector<rclcpp::Parameter> parameters)
  {
    dynamicParametersCallback(parameters);
  }

  int getCoarseSearchResolution()
  {
    return _coarse_search_resolution;
  }

  int getMaxIterations()
  {
    return _max_iterations;
  }

  int getMaxOnApproachIterations()
  {
    return _max_on_approach_iterations;
  }

  nav2_smac_planner::GoalHeadingMode getGoalHeadingMode()
  {
    return _goal_heading_mode;
  }
};

// SMAC smoke tests for plugin-level issues rather than algorithms
// (covered by more extensively testing in other files)
// System tests in nav2_system_tests will actually plan with this work

TEST(SmacTest, test_smac_lattice)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeLattice =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacLatticeTest");
  nodeLattice->declare_parameter("test.debug_visualizations", rclcpp::ParameterValue(true));

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto dummy_cancel_checker = []() {
      return false;
    };

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  goal.pose.orientation.w = 1.0;
  auto planner = std::make_unique<LatticeWrap>();
  try {
    // invalid goal heading mode
    nodeLattice->declare_parameter("test.goal_heading_mode", std::string("UNKNOWN"));
    nodeLattice->set_parameter(rclcpp::Parameter("test.goal_heading_mode", std::string("UNKNOWN")));
    EXPECT_THROW(planner->configure(nodeLattice, "test", nullptr, costmap_ros), std::runtime_error);
    nodeLattice->set_parameter(rclcpp::Parameter("test.goal_heading_mode", std::string("DEFAULT")));

    // invalid Configuration resolution
    nodeLattice->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", -1));
    nodeLattice->set_parameter(rclcpp::Parameter("test.max_iterations", -1));
    nodeLattice->set_parameter(rclcpp::Parameter("test.max_on_approach_iterations", -1));

    EXPECT_NO_THROW(planner->configure(nodeLattice, "test", nullptr, costmap_ros));
    EXPECT_EQ(planner->getCoarseSearchResolution(), 1);
    EXPECT_EQ(planner->getMaxIterations(), std::numeric_limits<int>::max());
    EXPECT_EQ(planner->getMaxOnApproachIterations(), std::numeric_limits<int>::max());


    // Valid configuration
    nodeLattice->set_parameter(rclcpp::Parameter("test.max_iterations", 1000000));
    nodeLattice->set_parameter(rclcpp::Parameter("test.max_on_approach_iterations", 1000));

    // Coarse search resolution will throw, not multiple of number of heading(16 default)
    nodeLattice->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", 3));
    EXPECT_THROW(planner->configure(nodeLattice, "test", nullptr, costmap_ros), std::runtime_error);

    // Valid configuration
    nodeLattice->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", 4));
    // Expect to throw due to invalid prims file in param
    planner->configure(nodeLattice, "test", nullptr, costmap_ros);
    EXPECT_EQ(planner->getCoarseSearchResolution(), 4);
  } catch (...) {
  }
  planner->activate();

  try {
    planner->createPlan(start, goal, dummy_cancel_checker);
  } catch (...) {
  }

  // corner case where the start and goal are on the same cell
  goal.pose.position.x = 0.01;
  goal.pose.position.y = 0.01;

  nav_msgs::msg::Path plan = planner->createPlan(start, goal, dummy_cancel_checker);
  EXPECT_EQ(plan.poses.size(), 1);  // single point path

  planner->deactivate();
  planner->cleanup();

  planner.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  costmap_ros.reset();
  nodeLattice.reset();
}

TEST(SmacTest, test_smac_lattice_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeLattice =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacLatticeTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<LatticeWrap>();
  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(nodeLattice, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    nodeLattice->get_node_base_interface(), nodeLattice->get_node_topics_interface(),
    nodeLattice->get_node_graph_interface(),
    nodeLattice->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.max_iterations", -1),
      rclcpp::Parameter("test.cache_obstacle_heuristic", true),
      rclcpp::Parameter("test.reverse_penalty", 5.0),
      rclcpp::Parameter("test.change_penalty", 1.0),
      rclcpp::Parameter("test.non_straight_penalty", 2.0),
      rclcpp::Parameter("test.cost_penalty", 2.0),
      rclcpp::Parameter("test.retrospective_penalty", 0.2),
      rclcpp::Parameter("test.analytic_expansion_ratio", 4.0),
      rclcpp::Parameter("test.max_planning_time", 10.0),
      rclcpp::Parameter("test.lookup_table_size", 30.0),
      rclcpp::Parameter("test.smooth_path", false),
      rclcpp::Parameter("test.analytic_expansion_max_length", 42.0),
      rclcpp::Parameter("test.tolerance", 42.0),
      rclcpp::Parameter("test.rotation_penalty", 42.0),
      rclcpp::Parameter("test.max_on_approach_iterations", 42),
      rclcpp::Parameter("test.terminal_checking_interval", 42),
      rclcpp::Parameter("test.allow_reverse_expansion", true)});

  try {
    // All of these params will re-init A* which will involve loading the control set file
    // which will cause an exception because the file does not exist. This will cause an
    // expected failure preventing parameter updates from being successfully processed
    rclcpp::spin_until_future_complete(
      nodeLattice->get_node_base_interface(),
      results);
  } catch (...) {
  }
  // test edge cases Goal heading mode, make sure we don't reset the goal when invalid
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("test.goal_heading_mode", std::string("BIDIRECTIONAL")));
  parameters.push_back(rclcpp::Parameter("test.goal_heading_mode", std::string("invalid")));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getGoalHeadingMode(), nav2_smac_planner::GoalHeadingMode::BIDIRECTIONAL);

  // test coarse resolution edge cases.
  // Negative coarse search resolution
  parameters.clear();
  parameters.push_back(rclcpp::Parameter("test.coarse_search_resolution", -1));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);

  // test value when coarse resolution
  // is not multiple number_of_headings
  parameters.clear();
  parameters.push_back(rclcpp::Parameter("test.coarse_search_resolution", 5));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);

  // Similar modulous test but when the issue is from the  number
  // of heading, test output includes number of heading 15
  parameters.clear();

  parameters.push_back(rclcpp::Parameter("test.coarse_search_resolution", 4));
  parameters.push_back(rclcpp::Parameter("test.lattice_filepath",
    ament_index_cpp::get_package_share_directory("nav2_smac_planner") +
    "/sample_primitives/test/output.json"));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);


  // So instead, let's call manually on a change
  parameters.clear();
  parameters.push_back(rclcpp::Parameter("test.lattice_filepath", std::string("HI")));
  EXPECT_THROW(planner->callDynamicParams(parameters), std::runtime_error);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
