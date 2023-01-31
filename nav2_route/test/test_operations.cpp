// Copyright (c) 2023, Samsung Research America
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
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_route/operations_manager.hpp"
#include "nav2_route/types.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

TEST(OperationsManagerTest, test_lifecycle)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  OperationsManager manager(node);
}

TEST(OperationsManagerTest, test_find_operations)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  OperationsManager manager(node);

  Node node2;
  EdgePtr ent{nullptr}, exit{nullptr};

  // No valid nodes or edges, should fail to find anything but not crash
  EXPECT_EQ(manager.findGraphOperationsToProcess(&node2, ent, exit).size(), 0u);

  // Still shouldn't find anything, but without nullptrs so can evaluate
  DirectionalEdge enter, exit2;
  EXPECT_EQ(manager.findGraphOperationsToProcess(&node2, &enter, &enter).size(), 0u);

  // Try again with some operations in the node and edge (2x-ed)
  Operation op, op2;
  op.type = "test";
  op.trigger = OperationTrigger::ON_ENTER;
  enter.operations.push_back(op);
  node2.operations.push_back(op);
  op2.type = "test2";
  op2.trigger = OperationTrigger::ON_EXIT;
  exit2.operations.push_back(op2);
  EXPECT_EQ(manager.findGraphOperationsToProcess(&node2, &enter, &exit2).size(), 3u);

  // Again, but now the triggers are inverted, so shouldn't be returned except node2
  enter.operations[0].trigger = OperationTrigger::ON_EXIT;
  exit2.operations[0].trigger = OperationTrigger::ON_ENTER;
  EXPECT_EQ(manager.findGraphOperationsToProcess(&node2, &enter, &exit2).size(), 1u);
}

TEST(OperationsManagerTest, test_find_operations_failure2)
{
  // This plugin does not exist
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  node->declare_parameter("operations", rclcpp::ParameterValue(std::vector<std::string>{"hi"}));
  EXPECT_THROW(OperationsManager manager(node), pluginlib::PluginlibException);
}

TEST(OperationsManagerTest, test_processing_fail)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");

  // No operation plugins to trigger
  node->declare_parameter("operations", rclcpp::ParameterValue(std::vector<std::string>{}));
  OperationsManager manager(node);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;

  // Should trigger nothing
  auto result = manager.process(true, state, route, pose);
  EXPECT_EQ(result.operations_triggered.size(), 0u);
}

TEST(OperationsManagerTest, test_processing_speed_on_status)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  OperationsManager manager(node);

  bool got_msg = false;
  nav2_msgs::msg::SpeedLimit my_msg;
  auto sub = node->create_subscription<nav2_msgs::msg::SpeedLimit>(
    "speed_limit",
    rclcpp::QoS(1),
    [&, this](nav2_msgs::msg::SpeedLimit msg) {got_msg = true; my_msg = msg;});

  Node node2;
  DirectionalEdge enter;
  float limit = 0.5f;
  enter.metadata.setValue<float>("speed_limit", limit);
  geometry_msgs::msg::PoseStamped pose;
  Route route;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;

  // No status change, shouldn't do anything
  OperationsResult result = manager.process(false, state, route, pose);
  EXPECT_FALSE(result.reroute);
  EXPECT_EQ(result.operations_triggered.size(), 0u);

  // Status change, may now trigger the only plugin
  result = manager.process(true, state, route, pose);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_EQ(result.operations_triggered[0], std::string("AdjustSpeedLimit"));
  rclcpp::Rate r(10);
  r.sleep();

  // Check values are correct
  EXPECT_TRUE(got_msg);
  EXPECT_TRUE(my_msg.percentage);
  EXPECT_NEAR(my_msg.speed_limit, 0.5f, 0.001f);
}

TEST(OperationsManagerTest, test_rerouting_service_on_query)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  auto node_int = std::make_shared<rclcpp::Node>("my_node2");

  // Enable rerouting service, which conducts on query (not status change)
  node->declare_parameter(
    "operations", rclcpp::ParameterValue(std::vector<std::string>{"ReroutingService"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "ReroutingService.plugin",
    rclcpp::ParameterValue(std::string{"nav2_route::ReroutingService"}));
  OperationsManager manager(node);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;

  // Should trigger, either way!
  auto result = manager.process(false, state, route, pose);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
  result = manager.process(true, state, route, pose);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);

  auto srv_client =
    nav2_util::ServiceClient<std_srvs::srv::Trigger>(
    "ReroutingService/reroute", node_int);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = srv_client.invoke(req, std::chrono::nanoseconds(1000000000));
  EXPECT_TRUE(resp->success);

  // Check values are correct after service call
  result = manager.process(true, state, route, pose);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_TRUE(result.reroute);

  // and resets
  result = manager.process(false, state, route, pose);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
}
