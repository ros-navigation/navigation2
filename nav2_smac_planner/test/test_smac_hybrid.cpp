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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "nav2_smac_planner/smac_planner_2d.hpp"

// Simple wrapper to be able to call a private member
class HybridWrap : public nav2_smac_planner::SmacPlannerHybrid
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

TEST(SmacTest, test_smac_se2)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeSE2 =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSE2Test");
  nodeSE2->declare_parameter("test.debug_visualizations", rclcpp::ParameterValue(true));

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  nodeSE2->declare_parameter("test.downsample_costmap", true);
  nodeSE2->set_parameter(rclcpp::Parameter("test.downsample_costmap", true));
  nodeSE2->declare_parameter("test.downsampling_factor", 2);
  nodeSE2->set_parameter(rclcpp::Parameter("test.downsampling_factor", 2));

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
  auto planner = std::make_unique<HybridWrap>();

  // invalid goal heading mode
  nodeSE2->declare_parameter("test.goal_heading_mode", std::string("UNKNOWN"));
  nodeSE2->set_parameter(rclcpp::Parameter("test.goal_heading_mode", std::string("UNKNOWN")));
  EXPECT_THROW(planner->configure(nodeSE2, "test", nullptr, costmap_ros), std::runtime_error);
  nodeSE2->set_parameter(rclcpp::Parameter("test.goal_heading_mode", std::string("DEFAULT")));

  // Invalid motion model should not change the default value
  nodeSE2->set_parameter(rclcpp::Parameter("test.motion_model_for_search", std::string("invalid")));
  EXPECT_THROW(planner->configure(nodeSE2, "test", nullptr, costmap_ros), std::runtime_error);
  nodeSE2->set_parameter(rclcpp::Parameter("test.motion_model_for_search", std::string("DUBIN")));

    // invalid coarse search resolution
  nodeSE2->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", -1));
  nodeSE2->set_parameter(rclcpp::Parameter("test.max_on_approach_iterations", -1));
  nodeSE2->set_parameter(rclcpp::Parameter("test.max_iterations", -1));
  EXPECT_NO_THROW(planner->configure(nodeSE2, "test", nullptr, costmap_ros));

  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);
  EXPECT_EQ(planner->getMaxIterations(), std::numeric_limits<int>::max());
  EXPECT_EQ(planner->getMaxOnApproachIterations(), std::numeric_limits<int>::max());


  // valid Configuration
  nodeSE2->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", 1));
  nodeSE2->set_parameter(rclcpp::Parameter("test.max_on_approach_iterations", 1000));
  nodeSE2->set_parameter(rclcpp::Parameter("test.max_iterations", 1000000));
  EXPECT_NO_THROW(planner->configure(nodeSE2, "test", nullptr, costmap_ros));


  // angle_quantizations not multiple of coarse search resolution would trigger a throw
  nodeSE2->set_parameter(rclcpp::Parameter("test.angle_quantization_bins", 72));
  nodeSE2->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", 5));
  EXPECT_THROW(planner->configure(nodeSE2, "test", nullptr, costmap_ros), std::runtime_error);

  // valid configuration
  nodeSE2->set_parameter(rclcpp::Parameter("test.coarse_search_resolution", 4));
  EXPECT_NO_THROW(planner->configure(nodeSE2, "test", nullptr, costmap_ros));

  planner->configure(nodeSE2, "test", nullptr, costmap_ros);
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
  nodeSE2.reset();
}

TEST(SmacTest, test_smac_se2_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeSE2 =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSE2Test");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<HybridWrap>();
  planner->configure(nodeSE2, "test", nullptr, costmap_ros);
  planner->activate();

  nodeSE2->declare_parameter("resolution", 0.05);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    nodeSE2->get_node_base_interface(), nodeSE2->get_node_topics_interface(),
    nodeSE2->get_node_graph_interface(),
    nodeSE2->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.downsample_costmap", true),
      rclcpp::Parameter("test.downsampling_factor", 2),
      rclcpp::Parameter("test.angle_quantization_bins", 72),
      rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.max_iterations", -1),
      rclcpp::Parameter("test.minimum_turning_radius", 1.0),
      rclcpp::Parameter("test.cache_obstacle_heuristic", true),
      rclcpp::Parameter("test.reverse_penalty", 5.0),
      rclcpp::Parameter("test.change_penalty", 1.0),
      rclcpp::Parameter("test.non_straight_penalty", 2.0),
      rclcpp::Parameter("test.cost_penalty", 2.0),
      rclcpp::Parameter("test.tolerance", 0.2),
      rclcpp::Parameter("test.retrospective_penalty", 0.2),
      rclcpp::Parameter("test.analytic_expansion_ratio", 4.0),
      rclcpp::Parameter("test.max_planning_time", 10.0),
      rclcpp::Parameter("test.lookup_table_size", 30.0),
      rclcpp::Parameter("test.smooth_path", false),
      rclcpp::Parameter("test.analytic_expansion_max_length", 42.0),
      rclcpp::Parameter("test.max_on_approach_iterations", 42),
      rclcpp::Parameter("test.terminal_checking_interval", 42),
      rclcpp::Parameter("test.motion_model_for_search", std::string("REEDS_SHEPP")),
      rclcpp::Parameter("test.goal_heading_mode", std::string("BIDIRECTIONAL")),
      rclcpp::Parameter("test.coarse_search_resolution", -1)});

  rclcpp::spin_until_future_complete(
    nodeSE2->get_node_base_interface(),
    results);

  EXPECT_EQ(nodeSE2->get_parameter("test.downsample_costmap").as_bool(), true);
  EXPECT_EQ(nodeSE2->get_parameter("test.downsampling_factor").as_int(), 2);
  EXPECT_EQ(nodeSE2->get_parameter("test.angle_quantization_bins").as_int(), 72);
  EXPECT_EQ(nodeSE2->get_parameter("test.allow_unknown").as_bool(), false);
  EXPECT_EQ(nodeSE2->get_parameter("test.max_iterations").as_int(), -1);
  EXPECT_EQ(nodeSE2->get_parameter("test.minimum_turning_radius").as_double(), 1.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.cache_obstacle_heuristic").as_bool(), true);
  EXPECT_EQ(nodeSE2->get_parameter("test.reverse_penalty").as_double(), 5.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.change_penalty").as_double(), 1.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.non_straight_penalty").as_double(), 2.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.cost_penalty").as_double(), 2.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.retrospective_penalty").as_double(), 0.2);
  EXPECT_EQ(nodeSE2->get_parameter("test.tolerance").as_double(), 0.2);
  EXPECT_EQ(nodeSE2->get_parameter("test.analytic_expansion_ratio").as_double(), 4.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.smooth_path").as_bool(), false);
  EXPECT_EQ(nodeSE2->get_parameter("test.max_planning_time").as_double(), 10.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.lookup_table_size").as_double(), 30.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.analytic_expansion_max_length").as_double(), 42.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.max_on_approach_iterations").as_int(), 42);
  EXPECT_EQ(nodeSE2->get_parameter("test.terminal_checking_interval").as_int(), 42);
  EXPECT_EQ(
    nodeSE2->get_parameter("test.motion_model_for_search").as_string(),
    std::string("REEDS_SHEPP"));
  EXPECT_EQ(
    nodeSE2->get_parameter("test.goal_heading_mode").as_string(),
    std::string("BIDIRECTIONAL"));

  auto results2 = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("resolution", 0.2)});
  rclcpp::spin_until_future_complete(
    nodeSE2->get_node_base_interface(),
    results2);
  EXPECT_EQ(nodeSE2->get_parameter("resolution").as_double(), 0.2);
  EXPECT_EQ(nodeSE2->get_parameter("test.coarse_search_resolution").as_int(), -1);
  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);

  // test coarse resolution edge cases. Consider when coarse resolution
  // is not multiple of angle bin quantization(72)
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("test.coarse_search_resolution", 7));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);

  // same test as before but the error comes from the angular bin
  parameters.clear();
  parameters.push_back(rclcpp::Parameter("test.coarse_search_resolution", 4));
  parameters.push_back(rclcpp::Parameter("test.angle_quantization_bins", 87));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getCoarseSearchResolution(), 1);

  // test invalid goal heading mode does not modify current
  // goal heading mode
  parameters.clear();
  parameters.push_back(rclcpp::Parameter("test.goal_heading_mode", std::string("BIDIRECTIONAL")));
  parameters.push_back(rclcpp::Parameter("test.goal_heading_mode", std::string("invalid")));
  EXPECT_NO_THROW(planner->callDynamicParams(parameters));
  EXPECT_EQ(planner->getGoalHeadingMode(), nav2_smac_planner::GoalHeadingMode::BIDIRECTIONAL);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
