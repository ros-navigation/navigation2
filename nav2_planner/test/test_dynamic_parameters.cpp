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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_planner/planner_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

TEST(WPTest, test_dynamic_parameters)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  auto param_handler_ = std::make_unique<nav2_planner::ParameterHandler>(
    node, node->get_logger());
  param_handler_->activate();
  auto params_ = param_handler_->getParams();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("expected_planner_frequency", 100.0),
      rclcpp::Parameter("allow_partial_planning", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->max_planner_duration, 0.01);
  EXPECT_TRUE(params_->partial_plan_allowed);

  // test invalid value, should be rejected
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("expected_planner_frequency", -1.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->max_planner_duration, 0.01);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("allow_partial_planning", false)});
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);
  EXPECT_FALSE(params_->partial_plan_allowed);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
