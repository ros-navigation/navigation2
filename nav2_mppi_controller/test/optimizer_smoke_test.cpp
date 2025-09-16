// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

// Smoke tests the optimizer

class OptimizerSuite : public ::testing::TestWithParam<std::tuple<std::string,
    std::vector<std::string>, bool>> {};

TEST_P(OptimizerSuite, OptimizerTest) {
  auto [motion_model, critics, consider_footprint] = GetParam();

  int batch_size = 400;
  int time_steps = 15;
  unsigned int path_points = 50u;
  int iteration_count = 1;
  double lookahead_distance = 10.0;

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  TestPose start_pose = costmap_settings.getCenterPose();
  double path_step = costmap_settings.resolution;

  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
    lookahead_distance, motion_model, consider_footprint};

  unsigned int offset = 4;
  unsigned int obstacle_size = offset * 2;

  unsigned char obstacle_cost = 250;

  auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

  obst_x = obst_x - offset;
  obst_y = obst_y - offset;
  addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});

  printInfo(optimizer_settings, path_settings, critics);
  auto node = getDummyNode(optimizer_settings, critics);
  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  EXPECT_NO_THROW(optimizer->evalControl(pose, velocity, path, dummy_goal_checker));
}

INSTANTIATE_TEST_SUITE_P(
  OptimizerTests,
  OptimizerSuite,
  ::testing::Values(
    std::make_tuple(
      "Omni",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"}, {"PathAlignCritic"},
          {"TwirlingCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true),
    std::make_tuple(
      "DiffDrive",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"CostCritic"},
          {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true),
    std::make_tuple(
      "Ackermann",
      std::vector<std::string>(
        {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
          {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}}),
      true))
);
