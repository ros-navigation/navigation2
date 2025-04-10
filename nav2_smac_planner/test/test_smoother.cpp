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
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace nav2_smac_planner;  // NOLINT

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class SmootherWrapper : public nav2_smac_planner::Smoother
{
public:
  explicit SmootherWrapper(const SmootherParams & params)
  : nav2_smac_planner::Smoother(params)
  {}

  std::vector<PathSegment> findDirectionalPathSegmentsWrapper(nav_msgs::msg::Path path)
  {
    return findDirectionalPathSegments(path);
  }
};

TEST(SmootherTest, test_full_smoother)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSmootherTest");
  nav2_smac_planner::SmootherParams params;
  params.get(node, "test");
  double maxtime = 1.0;

  // Make smoother and costmap to smooth in
  auto smoother = std::make_unique<SmootherWrapper>(params);
  smoother->initialize(0.4 /*turning radius*/);

  nav2_costmap_2d::Costmap2D * costmap =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.05, 0.0, 0.0, 0);
  // island in the middle of lethal cost to cross
  for (unsigned int i = 20; i <= 30; ++i) {
    for (unsigned int j = 20; j <= 30; ++j) {
      costmap->setCost(i, j, 254);
    }
  }

  // Setup A* search to get path to smooth
  nav2_smac_planner::SearchInfo info;
  info.change_penalty = 0.05;
  info.non_straight_penalty = 1.05;
  info.reverse_penalty = 2.0;
  info.cost_penalty = 2.0;
  info.retrospective_penalty = 0.0;
  info.analytic_expansion_ratio = 3.5;
  info.minimum_turning_radius = 8;  // in grid coordinates 0.4/0.05
  info.analytic_expansion_max_length = 20.0;  // in grid coordinates
  unsigned int size_theta = 72;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid> a_star(
    nav2_smac_planner::MotionModel::REEDS_SHEPP, info);
  int max_iterations = 10000;
  float tolerance = 0.0;
  int terminal_checking_interval = 5000;
  double max_planning_time = 120.0;
  int num_it = 0;

  a_star.initialize(
    false, max_iterations,
    std::numeric_limits<int>::max(), terminal_checking_interval, max_planning_time, 401,
    size_theta);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmapi = costmap_ros->getCostmap();
  *costmapi = *costmap;

  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> checker =
    std::make_unique<nav2_smac_planner::GridCollisionChecker>(costmap_ros, size_theta, node);
  checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);

  auto dummy_cancel_checker = []() {
      return false;
    };

  // Create A* search to smooth
  a_star.setCollisionChecker(checker.get());
  a_star.setStart(5u, 5u, 0u);
  a_star.setGoal(45u, 45u, 36u);
  nav2_smac_planner::NodeHybrid::CoordinateVector path;
  EXPECT_TRUE(a_star.createPath(path, num_it, tolerance, dummy_cancel_checker));

  // Convert to world coordinates and get length to compare to smoothed length
  nav_msgs::msg::Path plan;
  plan.header.stamp = node->now();
  plan.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  double initial_length = 0.0;
  double x_m = path[path.size() - 1].x, y_m = path[path.size() - 1].y;
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = nav2_smac_planner::getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = nav2_smac_planner::getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
    initial_length += hypot(path[i].x - x_m, path[i].y - y_m);
    x_m = path[i].x;
    y_m = path[i].y;
  }

  // Check that we accurately detect that this path has a reversing segment
  auto path_segs = smoother->findDirectionalPathSegmentsWrapper(plan);
  EXPECT_TRUE(path_segs.size() == 2u || path_segs.size() == 3u);

  // Test smoother, should succeed with same number of points
  // and shorter overall length, while still being collision free.
  auto path_size_in = plan.poses.size();
  EXPECT_TRUE(smoother->smooth(plan, costmap, maxtime));
  EXPECT_EQ(plan.poses.size(), path_size_in);  // Should have same number of poses
  double length = 0.0;
  x_m = plan.poses[0].pose.position.x;
  y_m = plan.poses[0].pose.position.y;
  for (unsigned int i = 0; i != plan.poses.size(); i++) {
    // Should be collision free
    EXPECT_EQ(costmap->getCost(plan.poses[i].pose.position.x, plan.poses[i].pose.position.y), 0);
    length += hypot(plan.poses[i].pose.position.x - x_m, plan.poses[i].pose.position.y - y_m);
    x_m = plan.poses[i].pose.position.x;
    y_m = plan.poses[i].pose.position.y;
  }
  EXPECT_LT(length, initial_length);  // Should be shorter

  // Try again but with failure modes

  // Failure mode: not enough iterations to complete
  params.max_its_ = 0;
  auto smoother_bypass = std::make_unique<SmootherWrapper>(params);
  EXPECT_FALSE(smoother_bypass->smooth(plan, costmap, maxtime));
  params.max_its_ = 1;
  auto smoother_failure = std::make_unique<SmootherWrapper>(params);
  EXPECT_FALSE(smoother_failure->smooth(plan, costmap, maxtime));

  // Failure mode: Not enough time
  double max_no_time = 0.0;
  EXPECT_FALSE(smoother->smooth(plan, costmap, max_no_time));

  // Failure mode: invalid path and invalid orientation
  // make the end of the path invalid
  geometry_msgs::msg::PoseStamped lastPose = plan.poses.back();
  unsigned int mx, my;
  costmap->worldToMap(lastPose.pose.position.x, lastPose.pose.position.y, mx, my);
  for (unsigned int i = mx - 5; i <= mx + 5; ++i) {
    for (unsigned int j = my - 5; j <= my + 5; ++j) {
      costmap->setCost(i, j, 254);
    }
  }

  // duplicate last pose to make the orientation update fail
  plan.poses.push_back(lastPose);
  EXPECT_FALSE(smoother->smooth(plan, costmap, maxtime));
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.z, 1.0, 1e-3);
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.x, 0.0, 1e-3);
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.y, 0.0, 1e-3);
  EXPECT_NEAR(plan.poses.end()[-2].pose.orientation.w, 0.0, 1e-3);

  delete costmap;
  nav2_smac_planner::NodeHybrid::destroyStaticAssets();
}
