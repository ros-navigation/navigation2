// Copyright (c) 2025 Maurice Alexander Purnawan
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
#include "nav2_ros_common/service_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "gtest/gtest.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using nav2::ServiceServer;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestServiceServer : public ServiceServer<std_srvs::srv::Empty>
{
public:
  TestServiceServer(
    const std::string & name,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & provided_node,
    CallbackType callback)
  : ServiceServer(name, provided_node, callback)
  {}
};

TEST(ServiceServer, can_handle_all_introspection_modes)
{
  std::vector<std::string> introspection_modes = {
    "disabled", "metadata", "contents"
  };

  for (const auto & mode : introspection_modes) {
    int a = 0;
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_" + mode);
    node->declare_parameter("introspection_mode", mode);

    auto callback = [&a](const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        a = 1;
      };

    TestServiceServer server("empty_srv_" + mode, node, callback);

    server.on_activate();

    auto client_node = rclcpp::Node::make_shared("client_node_" + mode);
    auto client = client_node->create_client<std_srvs::srv::Empty>("empty_srv_" + mode);
    rclcpp::executors::SingleThreadedExecutor node_executor;
    node_executor.add_node(node->get_node_base_interface());
    rclcpp::executors::SingleThreadedExecutor client_node_executor;
    client_node_executor.add_node(client_node);

    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = client->async_send_request(req);
    node_executor.spin_some();
    client_node_executor.spin_some();
    ASSERT_EQ(a, 1);
  }
}


TEST(ServiceServer, rejects_requests_when_inactive)
{
  int a = 0;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_inactive");

  auto callback = [&a](
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
      a = 1;
    };

  TestServiceServer server("empty_srv_inactive", node, callback);

  ASSERT_FALSE(server.is_activated());

  auto client_node = rclcpp::Node::make_shared("client_node_inactive");
  auto client = client_node->create_client<std_srvs::srv::Empty>("empty_srv_inactive");
  rclcpp::executors::SingleThreadedExecutor node_executor;
  node_executor.add_node(node->get_node_base_interface());
  rclcpp::executors::SingleThreadedExecutor client_node_executor;
  client_node_executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  const auto send_request_and_spin = [&]() {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      client->async_send_request(req);
      node_executor.spin_some();
      client_node_executor.spin_some();
    };

  send_request_and_spin();
  EXPECT_EQ(a, 0);
}

TEST(ServiceServer, accepts_requests_after_activation)
{
  int a = 0;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_activate");

  auto callback = [&a](
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
      a++;
    };

  TestServiceServer server("empty_srv_activate", node, callback);

  auto client_node = rclcpp::Node::make_shared("client_node_activate");
  auto client = client_node->create_client<std_srvs::srv::Empty>("empty_srv_activate");
  rclcpp::executors::SingleThreadedExecutor node_executor;
  node_executor.add_node(node->get_node_base_interface());
  rclcpp::executors::SingleThreadedExecutor client_node_executor;
  client_node_executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  const auto send_request_and_spin = [&]() {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      client->async_send_request(req);
      node_executor.spin_some();
      client_node_executor.spin_some();
    };
  // Inactive
  send_request_and_spin();
  EXPECT_EQ(a, 0);
  // Activate
  server.on_activate();
  ASSERT_TRUE(server.is_activated());
  send_request_and_spin();
  EXPECT_EQ(a, 1);
  // Deactivate
  server.on_deactivate();
  ASSERT_FALSE(server.is_activated());
  a = 0;
  send_request_and_spin();
  EXPECT_EQ(a, 0);
}

TEST(ServiceServer, auto_activates_when_node_already_active)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node_auto_activate");
  node->configure();
  node->activate();

  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  int a = 0;
  auto callback = [&a](
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
      a = 1;
    };

  auto server = std::make_shared<ServiceServer<std_srvs::srv::Empty>>(
    "empty_srv_auto", node, callback);

  ASSERT_TRUE(server->is_activated());

  auto client_node = rclcpp::Node::make_shared("client_node_auto");
  auto client = client_node->create_client<std_srvs::srv::Empty>("empty_srv_auto");

  rclcpp::executors::SingleThreadedExecutor node_executor;
  node_executor.add_node(node->get_node_base_interface());
  rclcpp::executors::SingleThreadedExecutor client_node_executor;
  client_node_executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  client->async_send_request(req);
  node_executor.spin_some();
  client_node_executor.spin_some();

  EXPECT_EQ(a, 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
