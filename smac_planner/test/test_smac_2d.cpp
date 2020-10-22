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
#include "smac_planner/node_se2.hpp"
#include "smac_planner/a_star.hpp"
#include "smac_planner/collision_checker.hpp"
#include "smac_planner/smac_planner.hpp"
#include "smac_planner/smac_planner_2d.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

// SMAC smoke tests for plugin-level issues rather than algorithms
// (covered by more extensively testing in other files)
// System tests in nav2_system_tests will actually plan with this work

TEST(SmacTest, test_smac_2d)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node2D =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("Smac2DTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  node2D->declare_parameter("test.smooth_path", true);
  node2D->set_parameter(rclcpp::Parameter("test.smooth_path", true));
  node2D->declare_parameter("test.downsample_costmap", true);
  node2D->set_parameter(rclcpp::Parameter("test.downsample_costmap", true));
  node2D->declare_parameter("test.downsampling_factor", 2);
  node2D->set_parameter(rclcpp::Parameter("test.downsampling_factor", 2));

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  auto planner_2d = std::make_unique<smac_planner::SmacPlanner2D>();
  planner_2d->configure(node2D, "test", nullptr, costmap_ros);
  planner_2d->activate();
  try {
    planner_2d->createPlan(start, goal);
  } catch (...) {
  }

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  node2D.reset();
  costmap_ros.reset();
}
