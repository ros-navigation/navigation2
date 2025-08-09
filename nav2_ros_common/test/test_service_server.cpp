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
#include "gtest/gtest.h"

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
    const rclcpp::Node::SharedPtr & provided_node,
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
    auto node = rclcpp::Node::make_shared("test_node_" + mode);

    node->declare_parameter("introspection_mode", mode);

    auto callback = [&a](const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        a = 1;
      };

    TestServiceServer server("empty_srv_" + mode, node, callback);

    auto client_node = rclcpp::Node::make_shared("client_node_" + mode);
    auto client = client_node->create_client<std_srvs::srv::Empty>("empty_srv_" + mode);

    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = client->async_send_request(req);
    rclcpp::spin_some(node);
    rclcpp::spin_some(client_node);
    ASSERT_EQ(a, 1);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
