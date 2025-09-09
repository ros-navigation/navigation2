// Copyright (c) 2025 Open Navigation LLC
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
#include <memory>
#include <set>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/compute_and_track_route_action.hpp"

class ComputeAndTrackRouteActionServer
  : public TestActionServer<nav2_msgs::action::ComputeAndTrackRoute>
{
public:
  ComputeAndTrackRouteActionServer()
  : TestActionServer("compute_and_track_route")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputeAndTrackRoute>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::ComputeAndTrackRoute::Result>();
    result->execution_duration = rclcpp::Duration::from_seconds(0.1);
    goal_handle->succeed(result);
  }
};

class ComputeAndTrackRouteActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("follow_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));
    config_->blackboard->set("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::ComputeAndTrackRouteAction>(
          name, "compute_and_track_route", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ComputeAndTrackRouteAction>(
      "ComputeAndTrackRoute", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ComputeAndTrackRouteActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ComputeAndTrackRouteActionTestFixture::node_ = nullptr;
std::shared_ptr<ComputeAndTrackRouteActionServer>
ComputeAndTrackRouteActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * ComputeAndTrackRouteActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ComputeAndTrackRouteActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ComputeAndTrackRouteActionTestFixture::tree_ = nullptr;

TEST_F(ComputeAndTrackRouteActionTestFixture, test_tick_poses)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ComputeAndTrackRoute goal="{goal}" start="{start}" use_poses="true" use_start="true" execution_duration="{execution_duration}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new start and set it on blackboard
  geometry_msgs::msg::PoseStamped start;
  start.header.stamp = node_->now();
  start.pose.position.x = 2.0;
  config_->blackboard->set("start", start);

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", goal);

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, 1.0);
  EXPECT_EQ(action_server_->getCurrentGoal()->start.pose.position.x, 2.0);
  EXPECT_TRUE(action_server_->getCurrentGoal()->use_start);
  EXPECT_TRUE(action_server_->getCurrentGoal()->use_poses);

  builtin_interfaces::msg::Duration time1;
  EXPECT_TRUE(
    config_->blackboard->get<builtin_interfaces::msg::Duration>("execution_duration", time1));
  EXPECT_EQ(time1, rclcpp::Duration::from_seconds(0.1));

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal
  goal.pose.position.x = -2.5;
  config_->blackboard->set("goal", goal);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, -2.5);
}

TEST_F(ComputeAndTrackRouteActionTestFixture, test_tick_ids)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ComputeAndTrackRoute goal_id="10" start_id="20" execution_duration="{execution_duration}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal_id, 10);
  EXPECT_EQ(action_server_->getCurrentGoal()->start_id, 20);
  EXPECT_EQ(action_server_->getCurrentGoal()->start, geometry_msgs::msg::PoseStamped());
  EXPECT_EQ(action_server_->getCurrentGoal()->goal, geometry_msgs::msg::PoseStamped());
  EXPECT_FALSE(action_server_->getCurrentGoal()->use_start);
  EXPECT_FALSE(action_server_->getCurrentGoal()->use_poses);

  builtin_interfaces::msg::Duration time1;
  EXPECT_TRUE(
    config_->blackboard->get<builtin_interfaces::msg::Duration>("execution_duration", time1));
  EXPECT_EQ(time1, rclcpp::Duration::from_seconds(0.1));

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  ComputeAndTrackRouteActionTestFixture::action_server_ =
    std::make_shared<ComputeAndTrackRouteActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(ComputeAndTrackRouteActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
