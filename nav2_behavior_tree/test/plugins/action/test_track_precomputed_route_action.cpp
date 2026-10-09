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
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/track_precomputed_route_action.hpp"
#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_ros_common/node_thread.hpp"

using namespace std::chrono_literals;

using Action = nav2_msgs::action::TrackPrecomputedRoute;

class PrecomputedActionServer : public TestActionServer<Action>
{
public:
  PrecomputedActionServer()
  : TestActionServer("track_precomputed_route") {}

protected:
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle) override
  {
    auto result = std::make_shared<Action::Result>();
    result->execution_duration = rclcpp::Duration::from_seconds(0.1);
    if (handle->get_goal()->route.nodes.front().nodeid == 99) {
      result->error_code = Action::Result::REROUTE_REQUIRED;
      result->error_msg = "Route blocked";
      result->blocked_ids = {10, 20};
      handle->abort(result);
    } else {
      handle->succeed(result);
    }
  }
};

nav2_msgs::msg::Route makeRoute(uint16_t start)
{
  nav2_msgs::msg::Route route;
  nav2_msgs::msg::RouteNode node;
  node.nodeid = start;
  route.nodes.push_back(node);
  node.nodeid = 3;
  route.nodes.push_back(node);
  nav2_msgs::msg::RouteEdge edge;
  edge.edgeid = 20;
  route.edges.push_back(edge);
  return route;
}

TEST(TrackPrecomputedRouteBT, SendsGoalAndPropagatesResultAndBlockedIDs)
{
  auto server = std::make_shared<PrecomputedActionServer>();
  nav2::NodeThread server_thread(server);
  auto node = std::make_shared<nav2::LifecycleNode>("precomputed_bt_client");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("server_timeout", 20ms);
  blackboard->set("bt_loop_duration", 10ms);
  blackboard->set("wait_for_service_timeout", 1000ms);
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<nav2_behavior_tree::TrackPrecomputedRouteAction>(
    "TrackPrecomputedRoute",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_behavior_tree::TrackPrecomputedRouteAction>(
        name, "track_precomputed_route", config);
    });
  auto tree =
    factory.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <TrackPrecomputedRoute route="{route}"
          blocked_ids="{blocked}" error_code_id="{error}" error_msg="{message}"
          execution_duration="{duration}"/>
      </BehaviorTree>
    </root>)",
    blackboard);

  for (uint16_t id : {1, 99, 2}) {
    blackboard->set("route", makeRoute(id));
    auto status = BT::NodeStatus::RUNNING;
    auto deadline = std::chrono::steady_clock::now() + 3s;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
    }
    const bool blocked = id == 99;
    EXPECT_EQ(status, blocked ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS);
    ASSERT_NE(server->getCurrentGoal(), nullptr);
    EXPECT_EQ(server->getCurrentGoal()->route, makeRoute(id));
    EXPECT_EQ(blackboard->get<uint16_t>("error"),
      blocked ? Action::Result::REROUTE_REQUIRED : Action::Result::NONE);
    EXPECT_EQ(blackboard->get<std::vector<uint32_t>>("blocked"),
      blocked ? (std::vector<uint32_t>{10, 20}) : std::vector<uint32_t>());
    tree.haltTree();
  }
}

class InspectablePrecomputedAction : public nav2_behavior_tree::TrackPrecomputedRouteAction
{
public:
  using TrackPrecomputedRouteAction::TrackPrecomputedRouteAction;
  const nav2_msgs::action::TrackPrecomputedRoute::Goal & request() const {return goal_;}
  bool updated() const {return goal_updated_;}
};

TEST(TrackPrecomputedRouteBT, UpdatesAllGoalInputsBeforePreemption)
{
  auto server = std::make_shared<PrecomputedActionServer>();
  nav2::NodeThread server_thread(server);
  auto node = std::make_shared<nav2::LifecycleNode>("precomputed_bt_update_client");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("server_timeout", 20ms);
  blackboard->set("bt_loop_duration", 10ms);
  blackboard->set("wait_for_service_timeout", 1000ms);
  blackboard->set("route", makeRoute(1));
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<InspectablePrecomputedAction>(
    "TrackPrecomputedRoute",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<InspectablePrecomputedAction>(
        name, "track_precomputed_route", config);
    });
  auto tree =
    factory.createTreeFromText(
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <TrackPrecomputedRoute route="{route}"
          path="{path}" blocked_ids="{blocked}"/>
      </BehaviorTree>
    </root>)",
    blackboard);
  auto action = dynamic_cast<InspectablePrecomputedAction *>(tree.rootNode());
  ASSERT_NE(action, nullptr);
  action->on_tick();
  EXPECT_EQ(action->request().route, makeRoute(1));
  blackboard->set("route", makeRoute(2));
  nav_msgs::msg::Path stale_path;
  stale_path.poses.resize(2);
  blackboard->set("path", stale_path);
  blackboard->set("blocked", std::vector<uint32_t>{10});
  auto stale_feedback = std::make_shared<Action::Feedback>();
  stale_feedback->path = stale_path;
  action->on_wait_for_result(stale_feedback);
  EXPECT_TRUE(action->updated());
  EXPECT_EQ(action->request().route, makeRoute(2));
  EXPECT_TRUE(blackboard->get<nav_msgs::msg::Path>("path").poses.empty());
  EXPECT_TRUE(blackboard->get<std::vector<uint32_t>>("blocked").empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int status = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return status;
}
