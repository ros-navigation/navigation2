// Copyright (c) 2026 Yong Ling
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
#include <vector>

#include "gtest/gtest.h"
#include "nav2_route/route_server.hpp"
#include "nav2_ros_common/node_thread.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace nav2_route
{

class BlockingOperation : public RouteOperation
{
public:
  explicit BlockingOperation(bool fail)
  : fail_(fail) {}
  void configure(
    const nav2::LifecycleNode::SharedPtr,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>, const std::string &) override {}
  std::string getName() override {return "test_blocking_operation";}
  OperationResult perform(
    NodePtr, EdgePtr, EdgePtr, const Route &,
    const geometry_msgs::msg::PoseStamped &, const Metadata *) override
  {
    if (fail_) {
      throw nav2_core::OperationFailed("Test operation failed");
    }
    return {true, {10, 70000}};
  }

private:
  bool fail_;
};

class TestOperationsManager : public OperationsManager
{
public:
  TestOperationsManager(nav2::LifecycleNode::SharedPtr node, bool fail)
  : OperationsManager(node, nullptr)
  {
    query_operations_.push_back(std::make_shared<BlockingOperation>(fail));
  }
};

class TestRouteTracker : public RouteTracker
{
public:
  void useOperation(nav2::LifecycleNode::SharedPtr node, bool fail)
  {
    operations_manager_ = std::make_unique<TestOperationsManager>(node, fail);
  }
};

class PrecomputedRouteServer : public RouteServer
{
public:
  void start()
  {
    ASSERT_EQ(on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
    graph_.resize(3);
    for (unsigned int i = 0; i < graph_.size(); ++i) {
      graph_[i].nodeid = i + 1;
      graph_[i].coords.x = i * 5.0;
      id_to_graph_map_[i + 1] = i;
    }
    EdgeCost cost{1.0f, false};
    graph_[0].addEdge(cost, &graph_[1], 10);
    graph_[1].addEdge(cost, &graph_[2], 20);
    goal_intent_extractor_->setGraph(graph_, &id_to_graph_map_);
    ASSERT_EQ(on_activate(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
  }

  void stop()
  {
    on_deactivate(rclcpp_lifecycle::State());
    on_cleanup(rclcpp_lifecycle::State());
  }

  void pose(double x)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = now();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x;
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "test", true);
  }

  void clearGraph() {graph_.clear();}
  void clearTF() {tf_->clear();}

  void useOperation(bool fail)
  {
    route_tracker_.reset();
    set_parameter(rclcpp::Parameter("operations", std::vector<std::string>()));
    auto tracker = std::make_shared<TestRouteTracker>();
    tracker->configure(
      shared_from_this(), tf_, costmap_subscriber_, compute_and_track_route_server_,
      route_frame_, base_frame_);
    tracker->useOperation(shared_from_this(), fail);
    route_tracker_ = tracker;
  }

  bool replaceGraph()
  {
    auto request = std::make_shared<nav2_msgs::srv::SetRouteGraph::Request>();
    request->graph_filepath = nav2::get_package_share_directory("nav2_route") +
      "/graphs/aws_graph.geojson";
    auto response = std::make_shared<nav2_msgs::srv::SetRouteGraph::Response>();
    setRouteGraph(nullptr, request, response);
    return response->success;
  }
};

class TrackPrecomputedRouteTest : public ::testing::Test
{
protected:
  using Action = nav2_msgs::action::TrackPrecomputedRoute;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;

  void SetUp() override
  {
    server = std::make_shared<PrecomputedRouteServer>();
    server->start();
    server->pose(0.0);
    server_thread = std::make_unique<nav2::NodeThread>(server);
    node = std::make_shared<rclcpp::Node>("precomputed_route_client");
    client = rclcpp_action::create_client<Action>(node, "track_precomputed_route");
    ASSERT_TRUE(client->wait_for_action_server(2s));
  }

  void TearDown() override
  {
    server->stop();
    server_thread.reset();
    client.reset();
    node.reset();
    server.reset();
  }

  Handle::SharedPtr send(const Action::Goal & goal, bool follow = false)
  {
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [this, follow](
      Handle::SharedPtr, const std::shared_ptr<const Action::Feedback> feedback)
      {
        feedbacks.push_back(*feedback);
        if (follow && feedback->next_node_id) {
          server->pose((feedback->next_node_id - 1) * 5.0);
        }
      };
    auto future = client->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(node, future, 3s) != rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "Goal response timed out";
      return nullptr;
    }
    return future.get();
  }

  Handle::WrappedResult result(const Handle::SharedPtr & handle)
  {
    auto future = client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(node, future, 3s) != rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "Action result timed out";
      return {};
    }
    return future.get();
  }

  void waitForFeedback()
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto until = std::chrono::steady_clock::now() + 2s;
    while (feedbacks.empty() && std::chrono::steady_clock::now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    executor.remove_node(node);
    ASSERT_FALSE(feedbacks.empty());
  }

  Action::Goal goal(std::string id = "fleet-route")
  {
    Action::Goal request;
    request.route_id = id;
    request.start_node_id = 1;
    request.edge_ids = {10, 20};
    return request;
  }

  std::shared_ptr<PrecomputedRouteServer> server;
  std::unique_ptr<nav2::NodeThread> server_thread;
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<Action>::SharedPtr client;
  std::vector<Action::Feedback> feedbacks;
};

TEST_F(TrackPrecomputedRouteTest, CompletesExactSequenceAndPublishesPath)
{
  auto handle = send(goal(), true);
  ASSERT_NE(handle, nullptr);
  auto completed = result(handle);
  ASSERT_NE(completed.result, nullptr);
  EXPECT_EQ(completed.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(completed.result->route_id, "fleet-route");
  EXPECT_EQ(completed.result->error_code, Action::Result::NONE);
  ASSERT_FALSE(feedbacks.empty());
  const auto & first = feedbacks.front();
  EXPECT_EQ(first.route_id, "fleet-route");
  ASSERT_EQ(first.route.edges.size(), 2u);
  EXPECT_EQ(first.route.edges[0].edgeid, 10u);
  EXPECT_EQ(first.route.edges[1].edgeid, 20u);
  EXPECT_EQ(first.path.header.frame_id, "map");
  EXPECT_GT(first.path.poses.size(), 3u);
  EXPECT_EQ(feedbacks.back().last_node_id, 3u);
}

TEST_F(TrackPrecomputedRouteTest, RejectsInvalidRouteWithoutTracking)
{
  auto request = goal();
  request.edge_ids = {20, 10};
  auto handle = send(request);
  ASSERT_NE(handle, nullptr);
  auto rejected = result(handle);
  ASSERT_NE(rejected.result, nullptr);
  EXPECT_EQ(rejected.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(rejected.result->error_code, Action::Result::INVALID_ROUTE);
  EXPECT_EQ(rejected.result->route_id, request.route_id);
  EXPECT_TRUE(feedbacks.empty());
}

TEST_F(TrackPrecomputedRouteTest, CancelsActiveRouteAndReleasesGraph)
{
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  waitForFeedback();
  EXPECT_FALSE(server->replaceGraph());
  auto canceled = client->async_cancel_goal(handle);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, canceled, 3s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result(handle).code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(server->replaceGraph());
}

TEST_F(TrackPrecomputedRouteTest, PreemptsWithNewRouteAndCorrelationID)
{
  auto original = send(goal("old"));
  ASSERT_NE(original, nullptr);
  waitForFeedback();
  auto replacement = goal("new");
  replacement.edge_ids.clear();
  auto handle = send(replacement);
  ASSERT_NE(handle, nullptr);
  auto completed = result(handle);
  ASSERT_NE(completed.result, nullptr);
  EXPECT_EQ(completed.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(completed.result->route_id, "new");
  auto preempted = result(original);
  EXPECT_EQ(preempted.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(preempted.result, nullptr);
  EXPECT_EQ(preempted.result->route_id, "old");
}

TEST_F(TrackPrecomputedRouteTest, ReturnsRerouteRequiredWithoutLocalPlanning)
{
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  waitForFeedback();
  auto service = node->create_client<std_srvs::srv::Trigger>(
    "route_server/ReroutingService/reroute");
  ASSERT_TRUE(service->wait_for_service(2s));
  auto response = service->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, response, 3s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(response.get()->success);
  auto aborted = result(handle);
  ASSERT_NE(aborted.result, nullptr);
  EXPECT_EQ(aborted.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(aborted.result->error_code, Action::Result::REROUTE_REQUIRED);
  EXPECT_EQ(aborted.result->route_id, "fleet-route");
}

TEST_F(TrackPrecomputedRouteTest, AllowsPlanningButSerializesTrackingActions)
{
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  waitForFeedback();
  auto compute = rclcpp_action::create_client<nav2_msgs::action::ComputeRoute>(node,
      "compute_route");
  ASSERT_TRUE(compute->wait_for_action_server(2s));
  nav2_msgs::action::ComputeRoute::Goal request;
  request.start_id = 1;
  request.goal_id = 3;
  auto sent = compute->async_send_goal(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, sent, 3s), rclcpp::FutureReturnCode::SUCCESS);
  auto planned = compute->async_get_result(sent.get());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, planned, 3s),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(planned.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  auto tracking = rclcpp_action::create_client<nav2_msgs::action::ComputeAndTrackRoute>(
    node, "compute_and_track_route");
  ASSERT_TRUE(tracking->wait_for_action_server(2s));
  nav2_msgs::action::ComputeAndTrackRoute::Goal competing;
  competing.start_id = 1;
  competing.goal_id = 3;
  auto second = tracking->async_send_goal(competing);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, second, 3s),
      rclcpp::FutureReturnCode::SUCCESS);
  auto busy = tracking->async_get_result(second.get());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, busy, 3s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(busy.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(busy.get().result->error_msg, "Another route tracking request is active");
  auto canceled = client->async_cancel_goal(handle);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, canceled, 3s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result(handle).code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(TrackPrecomputedRouteTest, RejectsNewActionWhileComputedRouteIsTracking)
{
  using Computed = nav2_msgs::action::ComputeAndTrackRoute;
  auto tracking = rclcpp_action::create_client<Computed>(node, "compute_and_track_route");
  ASSERT_TRUE(tracking->wait_for_action_server(2s));
  std::promise<void> started;
  auto ready = started.get_future();
  bool notified = false;
  rclcpp_action::Client<Computed>::SendGoalOptions options;
  options.feedback_callback = [&started, &notified](
    rclcpp_action::ClientGoalHandle<Computed>::SharedPtr,
    const std::shared_ptr<const Computed::Feedback>)
    {
      if (!notified) {
        notified = true;
        started.set_value();
      }
    };
  Computed::Goal request;
  request.start_id = 1;
  request.goal_id = 3;
  auto sent = tracking->async_send_goal(request, options);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, sent, 3s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, ready, 3s), rclcpp::FutureReturnCode::SUCCESS);
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  auto busy = result(handle);
  ASSERT_NE(busy.result, nullptr);
  EXPECT_EQ(busy.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(busy.result->error_code, Action::Result::BUSY);
  EXPECT_EQ(busy.result->route_id, "fleet-route");
  auto canceled = tracking->async_cancel_goal(sent.get());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, canceled, 3s),
      rclcpp::FutureReturnCode::SUCCESS);
  auto stopped = tracking->async_get_result(sent.get());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, stopped, 3s),
      rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(stopped.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(TrackPrecomputedRouteTest, RejectsStartOutsideBoundaryRadius)
{
  server->pose(3.0);
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  auto rejected = result(handle);
  ASSERT_NE(rejected.result, nullptr);
  EXPECT_EQ(rejected.result->error_code, Action::Result::INVALID_ROUTE);
  EXPECT_TRUE(feedbacks.empty());
}

TEST_F(TrackPrecomputedRouteTest, ReportsMissingTF)
{
  server->clearTF();
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  auto rejected = result(handle);
  ASSERT_NE(rejected.result, nullptr);
  EXPECT_EQ(rejected.result->error_code, Action::Result::TF_ERROR);
}

TEST_F(TrackPrecomputedRouteTest, ReportsEmptyGraph)
{
  server->clearGraph();
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  auto rejected = result(handle);
  ASSERT_NE(rejected.result, nullptr);
  EXPECT_EQ(rejected.result->error_code, Action::Result::NO_VALID_GRAPH);
}

TEST_F(TrackPrecomputedRouteTest, ReturnsBlockedIDsWithoutTruncation)
{
  server->useOperation(false);
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  auto aborted = result(handle);
  ASSERT_NE(aborted.result, nullptr);
  EXPECT_EQ(aborted.result->error_code, Action::Result::REROUTE_REQUIRED);
  EXPECT_EQ(aborted.result->blocked_ids, (std::vector<uint32_t>{10, 70000}));
}

TEST_F(TrackPrecomputedRouteTest, ReportsOperationFailure)
{
  server->useOperation(true);
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  auto aborted = result(handle);
  ASSERT_NE(aborted.result, nullptr);
  EXPECT_EQ(aborted.result->error_code, Action::Result::OPERATION_FAILED);
}

TEST_F(TrackPrecomputedRouteTest, DeactivatesWhileTracking)
{
  auto handle = send(goal());
  ASSERT_NE(handle, nullptr);
  waitForFeedback();
  // TearDown must stop the active tracker before releasing graph and plugin objects.
}

}  // namespace nav2_route

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int status = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return status;
}
