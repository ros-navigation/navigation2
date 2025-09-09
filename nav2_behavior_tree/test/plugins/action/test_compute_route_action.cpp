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
#include "nav2_behavior_tree/plugins/action/compute_route_action.hpp"

class ComputeRouteActionServer : public TestActionServer<nav2_msgs::action::ComputeRoute>
{
public:
  ComputeRouteActionServer()
  : TestActionServer("compute_route")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputeRoute>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::ComputeRoute::Result>();
    result->path.poses.resize(2);
    result->path.poses[1].pose.position.x = goal->goal.pose.position.x;
    if (goal->use_start) {
      result->path.poses[0].pose.position.x = goal->start.pose.position.x;
    } else {
      result->path.poses[0].pose.position.x = 0.0;
    }
    result->route.nodes.resize(2);
    result->route.edges.resize(2);
    result->planning_time = rclcpp::Duration::from_seconds(0.1);
    goal_handle->succeed(result);
  }
};

class ComputeRouteActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("compute_path_to_pose_action_test_fixture");
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
        return std::make_unique<nav2_behavior_tree::ComputeRouteAction>(
          name, "compute_route", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ComputeRouteAction>(
      "ComputeRoute", builder);
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

  static std::shared_ptr<ComputeRouteActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ComputeRouteActionTestFixture::node_ = nullptr;
std::shared_ptr<ComputeRouteActionServer>
ComputeRouteActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * ComputeRouteActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ComputeRouteActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ComputeRouteActionTestFixture::tree_ = nullptr;

TEST_F(ComputeRouteActionTestFixture, test_IDs)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ComputeRoute start_id="7" goal_id="2" path="{path}" route="{route}" planning_time="{planning_time}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->start_id, 7);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal_id, 2);
  EXPECT_FALSE(action_server_->getCurrentGoal()->use_start);
  EXPECT_FALSE(action_server_->getCurrentGoal()->use_poses);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal, geometry_msgs::msg::PoseStamped());
  EXPECT_EQ(action_server_->getCurrentGoal()->start, geometry_msgs::msg::PoseStamped());

  // check if returned values are correct
  nav_msgs::msg::Path path;
  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, 0.0);
  EXPECT_EQ(path.poses[1].pose.position.x, 0.0);

  nav2_msgs::msg::Route route;
  EXPECT_TRUE(config_->blackboard->get<nav2_msgs::msg::Route>("route", route));
  EXPECT_EQ(route.nodes.size(), 2u);
  EXPECT_EQ(route.edges.size(), 2u);

  builtin_interfaces::msg::Duration time1;
  EXPECT_TRUE(config_->blackboard->get<builtin_interfaces::msg::Duration>("planning_time", time1));
  EXPECT_EQ(time1, rclcpp::Duration::from_seconds(0.1));

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST_F(ComputeRouteActionTestFixture, test_poses)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ComputeRoute goal="{goal}" start="{start}" use_poses="true" use_start="true" path="{path}" route="{route}" planning_time="{planning_time}"/>
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

  // check if returned values are correct
  nav_msgs::msg::Path path;
  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, 2.0);
  EXPECT_EQ(path.poses[1].pose.position.x, 1.0);

  nav2_msgs::msg::Route route;
  EXPECT_TRUE(config_->blackboard->get<nav2_msgs::msg::Route>("route", route));
  EXPECT_EQ(route.nodes.size(), 2u);
  EXPECT_EQ(route.edges.size(), 2u);

  builtin_interfaces::msg::Duration time1;
  EXPECT_TRUE(config_->blackboard->get<builtin_interfaces::msg::Duration>("planning_time", time1));
  EXPECT_EQ(time1, rclcpp::Duration::from_seconds(0.1));

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);

  // set new goal and new start
  goal.pose.position.x = -2.5;
  start.pose.position.x = -1.5;
  config_->blackboard->set("goal", goal);
  config_->blackboard->set("start", start);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(action_server_->getCurrentGoal()->goal.pose.position.x, -2.5);

  EXPECT_TRUE(config_->blackboard->get<nav_msgs::msg::Path>("path", path));
  EXPECT_EQ(path.poses.size(), 2u);
  EXPECT_EQ(path.poses[0].pose.position.x, -1.5);
  EXPECT_EQ(path.poses[1].pose.position.x, -2.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  ComputeRouteActionTestFixture::action_server_ =
    std::make_shared<ComputeRouteActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(ComputeRouteActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
