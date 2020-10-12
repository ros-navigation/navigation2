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
#include "nav2_util/lifecycle_node.hpp"
#include "smac_planner/a_star.hpp"
#include "smac_planner/smoother.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(SmootherTest, test_smoother)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node2D =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmootherTest");

  // create and populate costmap
  nav2_costmap_2d::Costmap2D * costmap = new nav2_costmap_2d::Costmap2D(100, 100, 0.05, 0, 0, 0);

  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap->setCost(i, j, 254);
    }
  }

  // compute path to use
  smac_planner::SearchInfo info;
  info.change_penalty = 1.2;
  info.non_straight_penalty = 1.4;
  info.reverse_penalty = 2.1;
  info.minimum_turning_radius = 4.0;  // in grid coordinates
  unsigned int size_theta = 72;
  smac_planner::AStarAlgorithm<smac_planner::NodeSE2> a_star(
    smac_planner::MotionModel::DUBIN, info);
  int max_iterations = 1000000;
  float tolerance = 0.0;
  int it_on_approach = 1000000000;
  int num_it = 0;
  a_star.initialize(false, max_iterations, it_on_approach);
  a_star.setFootprint(nav2_costmap_2d::Footprint(), true);
  a_star.createGraph(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), size_theta, costmap);
  a_star.setStart(10u, 10u, 0u);
  a_star.setGoal(80u, 80u, 40u);
  smac_planner::NodeSE2::CoordinateVector plan;
  EXPECT_TRUE(a_star.createPath(plan, num_it, tolerance));

  // populate our smoothing paths
  std::vector<Eigen::Vector2d> path;
  std::vector<Eigen::Vector2d> initial_path;
  for (unsigned int i = 0; i != plan.size(); i++) {
    path.push_back(Eigen::Vector2d(plan[i].x, plan[i].y));
    initial_path.push_back(Eigen::Vector2d(plan[i].x, plan[i].y));
  }

  smac_planner::OptimizerParams params;
  params.debug = true;
  params.get(node2D.get(), "test");

  smac_planner::SmootherParams smoother_params;
  smoother_params.get(node2D.get(), "test");
  smoother_params.max_curvature = 5.0;
  smoother_params.curvature_weight = 30.0;
  smoother_params.distance_weight = 0.0;
  smoother_params.smooth_weight = 00.0;
  smoother_params.costmap_weight = 0.025;

  smac_planner::Smoother smoother;
  smoother.initialize(params);
  smoother.smooth(path, costmap, smoother_params);

  // kept at the right size
  EXPECT_EQ(path.size(), 73u);

  for (unsigned int i = 1; i != path.size() - 1; i++) {
    // check distance between points is in a good range
    EXPECT_NEAR(
      hypot(path[i][0] - path[i + 1][0], path[i][1] - path[i + 1][1]), 1.407170, 0.5);
  }

  delete costmap;
}
