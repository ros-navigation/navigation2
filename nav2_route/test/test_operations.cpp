// Copyright (c) 2025, Open Navigation LLC
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
#include "nav2_core/route_exceptions.hpp"

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
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);
}

TEST(OperationsManagerTest, test_failed_plugins)
{
  // This plugin does not exist
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  node->declare_parameter("operations", rclcpp::ParameterValue(std::vector<std::string>{"hi"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  EXPECT_THROW(OperationsManager manager(node, costmap_subscriber), std::runtime_error);
}

TEST(OperationsManagerTest, test_find_operations)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node2;
  EdgePtr ent{nullptr}, exit{nullptr};

  // No valid nodes or edges, should fail to find anything but not crash
  EXPECT_EQ(manager.findGraphOperations(&node2, ent, exit).size(), 0u);

  // Still shouldn't find anything, but without nullptrs so can evaluate
  DirectionalEdge enter, exit2;
  EXPECT_EQ(manager.findGraphOperations(&node2, &enter, &enter).size(), 0u);

  // Try again with some operations in the node and edge (2x-ed)
  Operation op, op2;
  op.type = "test";
  op.trigger = OperationTrigger::ON_ENTER;
  enter.operations.push_back(op);
  op.trigger = OperationTrigger::NODE;
  node2.operations.push_back(op);
  op2.type = "test2";
  op2.trigger = OperationTrigger::ON_EXIT;
  exit2.operations.push_back(op2);
  EXPECT_EQ(manager.findGraphOperations(&node2, &enter, &exit2).size(), 3u);

  // Again, but now the triggers are inverted, so shouldn't be returned except node2
  enter.operations[0].trigger = OperationTrigger::ON_EXIT;
  exit2.operations[0].trigger = OperationTrigger::ON_ENTER;
  EXPECT_EQ(manager.findGraphOperations(&node2, &enter, &exit2).size(), 1u);

  // Now, should be empty
  node2.operations[0].trigger = OperationTrigger::ON_ENTER;
  EXPECT_EQ(manager.findGraphOperations(&node2, &enter, &exit2).size(), 0u);
}

TEST(OperationsManagerTest, test_find_operations_failure2)
{
  // This plugin does not exist
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  node->declare_parameter("operations", rclcpp::ParameterValue(std::vector<std::string>{"hi"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  EXPECT_THROW(OperationsManager manager(node, costmap_subscriber), std::runtime_error);
}

TEST(OperationsManagerTest, test_processing_fail)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");

  // No operation plugins to trigger
  node->declare_parameter("operations", rclcpp::ParameterValue(std::vector<std::string>{}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;
  ReroutingState info;

  // Should trigger nothing
  auto result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 0u);
}

TEST(OperationsManagerTest, test_processing_speed_on_status)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  bool got_msg = false;
  nav2_msgs::msg::SpeedLimit my_msg;
  auto sub = node->create_subscription<nav2_msgs::msg::SpeedLimit>(
    "speed_limit", rclcpp::QoS(10),
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
  ReroutingState info;

  // No status change, shouldn't do anything
  OperationsResult result = manager.process(false, state, route, pose, info);
  EXPECT_FALSE(result.reroute);
  EXPECT_EQ(result.operations_triggered.size(), 1u);  // ReroutingService
  EXPECT_EQ(result.operations_triggered[0], std::string("ReroutingService"));

  // Status change, may now trigger the only plugin
  result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 2u);
  EXPECT_EQ(result.operations_triggered[0], std::string("AdjustSpeedLimit"));
  EXPECT_EQ(result.operations_triggered[1], std::string("ReroutingService"));
  rclcpp::Rate r(10);
  r.sleep();

  // Check values are correct
  EXPECT_TRUE(got_msg);
  EXPECT_TRUE(my_msg.percentage);
  EXPECT_NEAR(my_msg.speed_limit, 0.5f, 0.001f);
}

TEST(OperationsManagerTest, test_rerouting_service_on_query)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("route_server");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  auto node_int = std::make_shared<rclcpp::Node>("my_node2");

  // Enable rerouting service, which conducts on query (not status change)
  node->declare_parameter(
    "operations", rclcpp::ParameterValue(std::vector<std::string>{"ReroutingService"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "ReroutingService.plugin",
    rclcpp::ParameterValue(std::string{"nav2_route::ReroutingService"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;
  ReroutingState info;

  // Should trigger, either way!
  auto result = manager.process(false, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
  result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);

  auto srv_client =
    nav2_util::ServiceClient<std_srvs::srv::Trigger>(
    "route_server/ReroutingService/reroute", node_int);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = srv_client.invoke(req, std::chrono::nanoseconds(1000000000));
  EXPECT_TRUE(resp->success);

  // Check values are correct after service call
  result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_TRUE(result.reroute);

  // and resets
  result = manager.process(false, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
}

TEST(OperationsManagerTest, test_trigger_event_on_graph)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  auto node_int = std::make_shared<rclcpp::Node>("my_node2");
  auto node_thread_int = std::make_unique<nav2_util::NodeThread>(node_int);

  // Enable trigger event operation, which conducts on node or edge change
  // when a graph object contains the request for opening a door only.
  // This tests the trigger event plugin, ON_GRAPH actions in the
  // Operations Manager as well as the route operations client.
  node->declare_parameter(
    "operations", rclcpp::ParameterValue(std::vector<std::string>{"OpenDoor"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "OpenDoor.plugin",
    rclcpp::ParameterValue(std::string{"nav2_route::TriggerEvent"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;
  Metadata mdata;
  ReroutingState info;

  // Setup some test operations
  Operation op, op2, op3;
  op.type = "test";
  op.trigger = OperationTrigger::NODE;

  op2.type = "OpenDoor";
  op2.trigger = OperationTrigger::NODE;

  op3.type = "OpenDoor";
  op3.trigger = OperationTrigger::NODE;
  std::string service_name = "open_door";
  std::string key = "service_name";
  op3.metadata.setValue<std::string>(key, service_name);

  // Should do nothing in the operations manager, throw
  node2.operations.push_back(op);
  EXPECT_THROW(manager.process(true, state, route, pose, info), nav2_core::OperationFailed);

  // Now, let's try a node that should make it through the operations manager but fail
  // because the proper service_name was provided neither in the parameter nor operation
  // metadata
  node2.operations.clear();
  node2.operations.push_back(op2);
  EXPECT_THROW(manager.process(true, state, route, pose, info), nav2_core::OperationFailed);

  // Now let's test what should actually work with a real service in the metadata
  node2.operations.clear();
  node2.operations.push_back(op3);

  // This should throw because this service is not yet available on wait_for_service
  EXPECT_THROW(manager.process(true, state, route, pose, info), nav2_core::OperationFailed);

  // Now, let's test with a real server that is really available for use
  bool got_srv = false;
  auto callback =
    [&](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) -> void
    {
      got_srv = true;
      response->success = true;
    };

  auto service = node_int->create_service<std_srvs::srv::Trigger>(service_name, callback);

  auto result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
  EXPECT_TRUE(got_srv);
}

TEST(OperationsManagerTest, test_trigger_event_on_graph_global_service)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  auto node_int = std::make_shared<rclcpp::Node>("my_node2");
  auto node_thread_int = std::make_unique<nav2_util::NodeThread>(node_int);

  // Set the global service to use instead of file settings for conflict testing
  node->declare_parameter(
    "operations", rclcpp::ParameterValue(std::vector<std::string>{"OpenDoor"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "OpenDoor.plugin",
    rclcpp::ParameterValue(std::string{"nav2_route::TriggerEvent"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "OpenDoor.service_name",
    rclcpp::ParameterValue(std::string{"hello_world"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;
  Metadata mdata;
  ReroutingState info;

  // Setup working case
  Operation op3;
  op3.type = "OpenDoor";
  op3.trigger = OperationTrigger::NODE;
  std::string service_name = "open_door";
  std::string key = "service_name";
  op3.metadata.setValue<std::string>(key, service_name);
  node2.operations.push_back(op3);

  // Setup a server for the node's metadata, to create conflict with the global setting
  // If there's a conflict, the file version wins due to more specificity
  bool got_srv = false;
  auto callback =
    [&](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) -> void
    {
      got_srv = true;
      response->success = true;
    };

  auto service = node_int->create_service<std_srvs::srv::Trigger>(service_name, callback);

  auto result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
  EXPECT_TRUE(got_srv);

  // Now, let's reset without the metadata and see that the global version is now called
  node2.operations.clear();
  Operation op4;
  op4.type = "OpenDoor";
  op4.trigger = OperationTrigger::NODE;
  node2.operations.push_back(op4);

  bool got_srv2 = false;
  auto callback2 =
    [&](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) -> void
    {
      got_srv2 = true;
      response->success = true;
    };

  auto service2 = node_int->create_service<std_srvs::srv::Trigger>("hello_world", callback2);

  OperationsManager manager2(node, costmap_subscriber);
  result = manager2.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  EXPECT_FALSE(result.reroute);
  EXPECT_TRUE(got_srv2);
}

TEST(OperationsManagerTest, test_trigger_event_on_graph_failures)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  auto node_int = std::make_shared<rclcpp::Node>("my_node2");

  // Enable trigger event operation, which conducts on node or edge change
  // when a graph object contains the request for opening a door only
  node->declare_parameter(
    "operations", rclcpp::ParameterValue(std::vector<std::string>{"OpenDoor"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "OpenDoor.plugin",
    rclcpp::ParameterValue(std::string{"nav2_route::TriggerEvent"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node2;
  DirectionalEdge enter;
  RouteTrackingState state;
  state.last_node = &node2;
  state.next_node = &node2;
  state.current_edge = &enter;
  geometry_msgs::msg::PoseStamped pose;
  Route route;
  ReroutingState info;

  // No operations, nothing should trigger even though status changed
  auto result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 0u);
  EXPECT_FALSE(result.reroute);

  // Setup some test operations
  Operation op, op2;
  op.type = "test";
  op2.type = "TriggerEvent";
  op.trigger = OperationTrigger::NODE;
  op2.trigger = OperationTrigger::NODE;

  // Should also do nothing, this type isn't a plugin type supported
  node2.operations.push_back(op);
  EXPECT_THROW(manager.process(true, state, route, pose, info), nav2_core::OperationFailed);

  // Make sure its using the provided plugin name NOT its type
  node2.operations.clear();
  node2.operations.push_back(op2);
  EXPECT_THROW(manager.process(true, state, route, pose, info), nav2_core::OperationFailed);
}


TEST(OperationsManagerTest, test_time_marker)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("operations_manager_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  node->declare_parameter(
    "operations", rclcpp::ParameterValue(std::vector<std::string>{"TimeMarker"}));
  nav2_util::declare_parameter_if_not_declared(
    node, "TimeMarker.plugin",
    rclcpp::ParameterValue(std::string{"nav2_route::TimeMarker"}));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  OperationsManager manager(node, costmap_subscriber);

  Node node1, node2, node3, node4;
  DirectionalEdge exit, enter, last;
  exit.start = &node1;
  exit.end = &node2;
  enter.start = &node2;
  enter.end = &node3;
  last.start = &node3;
  last.end = &node4;
  geometry_msgs::msg::PoseStamped pose;

  Route route;
  route.start_node = &node1;
  route.edges.push_back(&exit);
  route.edges.push_back(&enter);
  route.edges.push_back(&last);

  ReroutingState info;
  RouteTrackingState state;
  state.last_node = &node1;
  state.next_node = &node2;
  state.current_edge = &exit;
  state.route_edges_idx = 0;

  // No status change, shouldn't do anything ... even after some time
  OperationsResult result = manager.process(false, state, route, pose, info);
  EXPECT_FALSE(result.reroute);
  EXPECT_EQ(result.operations_triggered.size(), 0u);
  rclcpp::Rate r(1);
  r.sleep();
  result = manager.process(false, state, route, pose, info);
  EXPECT_FALSE(result.reroute);
  EXPECT_EQ(result.operations_triggered.size(), 0u);

  // Status change, may now trigger but state doesn't match
  // (new edge) so it won't update times on the first call
  result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);

  float time = 0.0f;
  time = enter.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_EQ(time, 0.0f);
  time = 0.0f;
  time = exit.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_EQ(time, 0.0f);

  rclcpp::Rate r2(1);
  r2.sleep();

  // The second time around after switching edges, should update the last edge's time
  state.last_node = &node2;
  state.next_node = &node3;
  state.current_edge = &enter;
  state.route_edges_idx = 1;
  result = manager.process(true, state, route, pose, info);
  EXPECT_EQ(result.operations_triggered.size(), 1u);
  time = 0.0f;
  time = exit.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_GT(time, 0.5f);
  time = 0.0f;
  time = enter.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_EQ(time, 0.0f);

  // Immediately call again on new edge, should also work with an edge change
  // But the last edge not completed should still remain empty
  state.last_node = &node3;
  state.next_node = &node4;
  state.current_edge = &last;
  state.route_edges_idx = 2;
  result = manager.process(true, state, route, pose, info);
  time = 0.0f;
  time = enter.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_GT(time, 1e-6f);
  time = 0.0f;
  time = last.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_EQ(time, 0.0f);

  // Check on terminal conditions
  rclcpp::Rate r3(1);
  r3.sleep();
  state.last_node = &node4;
  state.next_node = nullptr;
  state.current_edge = nullptr;
  state.route_edges_idx = 3;
  result = manager.process(true, state, route, pose, info);
  time = 0.0f;
  time = last.metadata.getValue<float>("abs_time_taken", time);
  EXPECT_GT(time, 0.5f);
}

// A test operation that does nothing to test `processType()`
class TestRouteOperations : public nav2_route::RouteOperation
{
public:
  void configure(
    const nav2_util::LifecycleNode::SharedPtr,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    const std::string &) override
  {
  }

  std::string getName() override {return "TestRouteOperation";}
  OperationResult perform(
    NodePtr,
    EdgePtr,
    EdgePtr,
    const Route &,
    const geometry_msgs::msg::PoseStamped &,
    const Metadata *) override {return OperationResult();}
};

TEST(OperationsTest, test_interface)
{
  TestRouteOperations op;
  EXPECT_EQ(op.processType(), nav2_route::RouteOperationType::ON_GRAPH);
}
