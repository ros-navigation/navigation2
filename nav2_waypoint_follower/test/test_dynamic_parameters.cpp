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
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(WPTest, test_dynamic_parameters)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  auto param_handler_ = std::make_unique<nav2_waypoint_follower::ParameterHandler>(
    node, node->get_logger());
  param_handler_->activate();
  auto params_ = param_handler_->getParams();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("loop_rate", 100),
      rclcpp::Parameter("stop_on_failure", false)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params_->loop_rate, 100);
  EXPECT_EQ(params_->stop_on_failure, false);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("loop_rate", 0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  // Invalid value should not be set
  EXPECT_EQ(params_->loop_rate, 100);

  node->deactivate();
  node->cleanup();
  node.reset();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
