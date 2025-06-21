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
#include "nav2_ros_common/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "gtest/gtest.h"

using nav2::ServiceClient;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestServiceClient : public ServiceClient<std_srvs::srv::Empty>
{
public:
  TestServiceClient(
    const std::string & name,
    const rclcpp::Node::SharedPtr & provided_node = rclcpp::Node::SharedPtr(),
    bool use_internal_executor = false)
  : ServiceClient(name, provided_node, use_internal_executor)
  {}

  string name() {return node_base_interface_->get_name();}
};

TEST(ServiceClient, can_ServiceClient_use_passed_in_node)
{
  std::vector<std::string> introspection_modes = {
    "disabled", "metadata", "contents"
  };
  for(const auto & mode : introspection_modes) {
    auto node = rclcpp::Node::make_shared("test_node" + mode);
    node->declare_parameter("service_introspection_mode", mode);
    TestServiceClient t("bar", node, true);
    ASSERT_EQ(t.name(), "test_node" + mode);
  }
}

TEST(ServiceClient, can_ServiceClient_invoke_in_callback)
{
  int a = 0;
  auto service_node = rclcpp::Node::make_shared("service_node");
  auto service = service_node->create_service<std_srvs::srv::Empty>(
    "empty_srv",
    [&a](std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
      a = 1;
    });
  auto srv_thread = std::thread([&]() {rclcpp::spin(service_node);});

  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto pub = pub_node->create_publisher<std_msgs::msg::Empty>(
    "empty_topic",
    rclcpp::QoS(1).transient_local());
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node);});

  auto sub_node = rclcpp::Node::make_shared("sub_node");
  ServiceClient<std_srvs::srv::Empty> client("empty_srv", sub_node, true);
  auto sub = sub_node->create_subscription<std_msgs::msg::Empty>(
    "empty_topic",
    rclcpp::QoS(1),
    [&client](std_msgs::msg::Empty::SharedPtr) {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      auto res = client.invoke(req);
    });

  pub->publish(std_msgs::msg::Empty());
  rclcpp::spin_some(sub_node);

  rclcpp::shutdown();
  srv_thread.join();
  pub_thread.join();
  ASSERT_EQ(a, 1);
}

TEST(ServiceClient, can_ServiceClient_timeout)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_node");
  TestServiceClient t("bar", node, true);
  rclcpp::spin_some(node);
  bool ready = t.wait_for_service(std::chrono::milliseconds(10));
  rclcpp::shutdown();
  ASSERT_EQ(ready, false);
}

TEST(ServiceClient, can_ServiceClient_async_call) {
  rclcpp::init(0, nullptr);

  int a = 0;
  bool callback_called = false;
  // Define service server
  auto service_node = rclcpp::Node::make_shared("service_node");
  auto service = service_node->create_service<std_srvs::srv::Empty>(
  "empty_srv",
    [&a](std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
      a = 1;
  });
  auto srv_thread = std::thread([&]() {rclcpp::spin(service_node);});
  // Define service client
  auto node = rclcpp::Node::make_shared("test_node");
  ServiceClient<std_srvs::srv::Empty> client("empty_srv", node, false);
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto callback =
    [&callback_called](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*future*/) {
      callback_called = true;
    };
  // Test async_call
  client.async_call(req, callback);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::spin_some(node);

  rclcpp::shutdown();
  srv_thread.join();
  ASSERT_EQ(a, 1);
  ASSERT_TRUE(callback_called);
}
