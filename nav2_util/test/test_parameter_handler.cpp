// Copyright (c) 2023 Open Navigation LLC
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

using namespace std::chrono_literals;

struct DummyParams
{
  double value = 0.0;
};

class TestParameterHandler : public nav2_util::ParameterHandler<DummyParams>
{
public:
  TestParameterHandler(const nav2::LifecycleNode::SharedPtr & node, rclcpp::Logger & logger)
  : ParameterHandler<DummyParams>(node, logger) {}

  bool validated = false;
  bool updated = false;

  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & /*parameters*/) override
  {
    validated = true;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void updateParametersCallback(const std::vector<rclcpp::Parameter> & /*parameters*/) override
  {
    updated = true;
  }
};

TEST(ParameterHandlerTest, Construction)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2::LifecycleNode>("test_node");
  rclcpp::Logger logger = node->get_logger();
  TestParameterHandler handler(node, logger);
  EXPECT_NE(handler.getParams(), nullptr);
  rclcpp::shutdown();
}

TEST(ParameterHandlerTest, ActivateAndDeactivate)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2::LifecycleNode>("test_node");
  rclcpp::Logger logger = node->get_logger();

  TestParameterHandler handler(node, logger);
  handler.activate();
  EXPECT_TRUE(handler.getMutex().try_lock());
  handler.getMutex().unlock();
  handler.deactivate();
  rclcpp::shutdown();
}

TEST(ParameterHandlerTest, DynamicCallbacksAreInvoked)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2::LifecycleNode>("test_node");
  rclcpp::Logger logger = node->get_logger();
  TestParameterHandler handler(node, logger);

  handler.activate();

  // Declare a parameter and change it to trigger callbacks
  node->declare_parameter("test_param", 1.0);
  node->set_parameter(rclcpp::Parameter("test_param", 2.0));

  // Manually invoke callbacks
  std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("test_param", 3.0)};
  handler.validateParameterUpdatesCallback(params);
  handler.updateParametersCallback(params);

  EXPECT_TRUE(handler.validated);
  EXPECT_TRUE(handler.updated);
  handler.deactivate();
  rclcpp::shutdown();
}
