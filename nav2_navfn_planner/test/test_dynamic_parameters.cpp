// Copyright (c) 2021, Samsung Research America
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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "rclcpp/rclcpp.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NavfnTest, testDynamicParameter)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("Navfntest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto planner =
    std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  planner->configure(node, "test", tf, costmap);
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", 1.0),
      rclcpp::Parameter("test.use_astar", true),
      rclcpp::Parameter("test.allow_unknown", true),
      rclcpp::Parameter("test.use_final_approach_orientation", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.tolerance").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.use_astar").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.allow_unknown").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_final_approach_orientation").as_bool(), true);
}
