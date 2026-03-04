// Copyright (c) 2026 Open Navigation LLC
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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "nav2_ros_common/qos_profiles.hpp"
#include "nav2_ros_common/subscription.hpp"

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(SubscriptionLatched, LatchedMessageReceivedAfterActivation)
{
  const std::string topic_name = "test_latched_topic";
  constexpr int kExpectedData = 42;

  auto node = std::make_shared<rclcpp::Node>("test_subscription_latched_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  std::promise<int> received_promise;
  std::future<int> received_future = received_promise.get_future();
  auto callback = [&received_promise](std_msgs::msg::Int32::ConstSharedPtr msg) {
      received_promise.set_value(msg->data);
    };

  auto subscription = std::make_shared<nav2::Subscription<std_msgs::msg::Int32>>(
    node,
    topic_name,
    callback,
    nav2::qos::LatchedSubscriptionQoS());
  // Transient subscription is activated on init.

  auto publisher = node->create_publisher<std_msgs::msg::Int32>(
    topic_name,
    nav2::qos::LatchedPublisherQoS());

  std_msgs::msg::Int32 msg;
  msg.data = kExpectedData;
  publisher->publish(msg);

  // Allow discovery and latched delivery
  for (int i = 0; i < 20; ++i) {
    executor.spin_some(50ms);
    std::this_thread::sleep_for(10ms);
  }


  // Spin until callback invoked or timeout (2s)
  auto status = received_future.wait_for(2s);
  for (int i = 0; i < 100 && status != std::future_status::ready; ++i) {
    executor.spin_some(20ms);
    status = received_future.wait_for(20ms);
  }

  ASSERT_EQ(status, std::future_status::ready)
    << "Latched message was not received after subscription activation";
  EXPECT_EQ(received_future.get(), kExpectedData);
}
