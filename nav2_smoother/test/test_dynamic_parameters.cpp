// Copyright (c) 2026, Pannaga Sudarshan
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

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_smoother/simple_smoother_parameter_handler.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"


TEST(SimpleSmootherParamTest, test_dynamic_parameters)
{
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  std::string plugin_name = "test";
  auto logger = node->get_logger();
  auto param_handler = std::make_unique<nav2_smoother::SimpleSmootherParameterHandler>(
    node, plugin_name, logger);
  param_handler->activate();
  auto params = param_handler->getParams();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Valid updates are applied
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", 0.5),
      rclcpp::Parameter("test.max_its", 10),
      rclcpp::Parameter("test.do_refinement", false)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(params->tolerance, 0.5);
  EXPECT_EQ(params->max_its, 10);
  EXPECT_FALSE(params->do_refinement);

  // Invalid tolerance is rejected, value unchanged
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", -1.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_EQ(params->tolerance, 0.5);

  // Invalid max_its is rejected, value unchanged
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.max_its", 0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_EQ(params->max_its, 10);

  // Negative weight is rejected, default unchanged
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.w_data", -0.1)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_EQ(params->w_data, 0.2);

  // Zero weight is valid
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.w_smooth", 0.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_EQ(params->w_smooth, 0.0);
}


TEST(SimpleSmootherParamTest, test_reject_while_smoothing)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test_node");
  std::string plugin_name = "test";
  auto logger = node->get_logger();
  auto param_handler = std::make_unique<nav2_smoother::SimpleSmootherParameterHandler>(
    node, plugin_name, logger);
  param_handler->activate();
  auto params = param_handler->getParams();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // A second thread holds the handler mutex, standing in for a smooth() in progress
  std::atomic<bool> release{false};
  std::thread holder([&]() {
      std::lock_guard<std::mutex> lock(param_handler->getMutex());
      while (!release) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Update arrives while "smoothing": must be rejected, value unchanged
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", 0.5)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_FALSE(results.get().successful);
  EXPECT_EQ(params->tolerance, 1e-10);

  // "Smoothing" finishes
  release = true;
  holder.join();

  // Same update now succeeds
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", 0.5)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_TRUE(results.get().successful);
  EXPECT_EQ(params->tolerance, 0.5);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
