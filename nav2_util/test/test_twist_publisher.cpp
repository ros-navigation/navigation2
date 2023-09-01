// Copyright (c) 2019 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_util/twist_publisher.hpp"
#include "nav2_util/lifecycle_utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using nav2_util::startup_lifecycle_nodes;
using nav2_util::reset_lifecycle_nodes;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(TwistPublisher, Unstamped)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_node", "");
  node->declare_parameter("enable_stamped_cmd_vel", false);

  node->configure();
  node->activate();

  auto vel_publisher = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", 1);
  ASSERT_EQ(vel_publisher->get_subscription_count(), 0);

  vel_publisher->on_activate();

  const geometry_msgs::msg::TwistStamped twist_stamped {};
  vel_publisher->publish(twist_stamped);
  node->deactivate();
  SUCCEED();
}
