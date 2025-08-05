// Copyright (c) 2024 Angsa Robotics
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
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_service.hpp"
#include "nav2_behavior_tree/plugins/action/remove_in_collision_goals_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"


class RemoveInCollisionGoalsSuccessService : public TestService<nav2_msgs::srv::GetCosts>
{
public:
  RemoveInCollisionGoalsSuccessService()
  : TestService("/global_costmap/get_cost_global_costmap")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
  {
    (void)request_header;
    (void)request;
    response->costs = {100, 50, 5, 254};
    response->success = true;
  }
};

class RemoveInCollisionGoalsFailureService : public TestService<nav2_msgs::srv::GetCosts>
{
public:
  RemoveInCollisionGoalsFailureService()
  : TestService("/local_costmap/get_cost_local_costmap")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
  {
    (void)request_header;
    (void)request;
    response->costs = {255, 50, 5, 254};
    response->success = false;
  }
};

class RemoveInCollisionGoalsTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("in_collision_goals_test_fixture");
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

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::RemoveInCollisionGoals>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::RemoveInCollisionGoals>(
      "RemoveInCollisionGoals", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    success_server_.reset();
    failure_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }
  static std::shared_ptr<RemoveInCollisionGoalsSuccessService> success_server_;
  static std::shared_ptr<RemoveInCollisionGoalsFailureService> failure_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr RemoveInCollisionGoalsTestFixture::node_ = nullptr;

BT::NodeConfiguration * RemoveInCollisionGoalsTestFixture::config_ = nullptr;
std::shared_ptr<RemoveInCollisionGoalsSuccessService>
RemoveInCollisionGoalsTestFixture::success_server_ = nullptr;
std::shared_ptr<RemoveInCollisionGoalsFailureService>
RemoveInCollisionGoalsTestFixture::failure_server_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> RemoveInCollisionGoalsTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> RemoveInCollisionGoalsTestFixture::tree_ = nullptr;

TEST_F(RemoveInCollisionGoalsTestFixture, test_tick_remove_in_collision_goals_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <RemoveInCollisionGoals service_name="/global_costmap/get_cost_global_costmap" input_goals="{goals}" output_goals="{goals}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Goals poses;
  poses.goals.resize(4);
  poses.goals[0].pose.position.x = 0.0;
  poses.goals[0].pose.position.y = 0.0;

  poses.goals[1].pose.position.x = 0.5;
  poses.goals[1].pose.position.y = 0.0;

  poses.goals[2].pose.position.x = 1.0;
  poses.goals[2].pose.position.y = 0.0;

  poses.goals[3].pose.position.x = 2.0;
  poses.goals[3].pose.position.y = 0.0;

  config_->blackboard->set("goals", poses);

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  // check that it removed the point in range
  nav_msgs::msg::Goals output_poses;
  EXPECT_TRUE(config_->blackboard->get("goals", output_poses));

  EXPECT_EQ(output_poses.goals.size(), 3u);
  EXPECT_EQ(output_poses.goals[0], poses.goals[0]);
  EXPECT_EQ(output_poses.goals[1], poses.goals[1]);
  EXPECT_EQ(output_poses.goals[2], poses.goals[2]);
}

TEST_F(RemoveInCollisionGoalsTestFixture,
  test_tick_remove_in_collision_goals_success_and_output_waypoint_statues)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <RemoveInCollisionGoals service_name="/global_costmap/get_cost_global_costmap"
                                  input_goals="{goals}" output_goals="{goals}"
                                  cost_threshold="253"
                                  input_waypoint_statuses="{waypoint_statuses}"
                                  output_waypoint_statuses="{waypoint_statuses}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Goals poses;
  poses.goals.resize(4);
  poses.goals[0].pose.position.x = 0.0;
  poses.goals[0].pose.position.y = 0.0;

  poses.goals[1].pose.position.x = 0.5;
  poses.goals[1].pose.position.y = 0.0;

  poses.goals[2].pose.position.x = 1.0;
  poses.goals[2].pose.position.y = 0.0;

  poses.goals[3].pose.position.x = 2.0;
  poses.goals[3].pose.position.y = 0.0;

  config_->blackboard->set("goals", poses);

  // create waypoint_statuses and set it on blackboard
  std::vector<nav2_msgs::msg::WaypointStatus> waypoint_statuses(poses.goals.size());
  for (size_t i = 0 ; i < waypoint_statuses.size() ; ++i) {
    waypoint_statuses[i].waypoint_pose = poses.goals[i];
    waypoint_statuses[i].waypoint_index = i;
  }
  config_->blackboard->set("waypoint_statuses", waypoint_statuses);

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  // check that it removed the point in range
  nav_msgs::msg::Goals output_poses;
  EXPECT_TRUE(config_->blackboard->get("goals", output_poses));

  EXPECT_EQ(output_poses.goals.size(), 3u);
  EXPECT_EQ(output_poses.goals[0], poses.goals[0]);
  EXPECT_EQ(output_poses.goals[1], poses.goals[1]);
  EXPECT_EQ(output_poses.goals[2], poses.goals[2]);

  // check the waypoint_statuses
  std::vector<nav2_msgs::msg::WaypointStatus> output_waypoint_statuses;
  EXPECT_TRUE(config_->blackboard->get("waypoint_statuses", output_waypoint_statuses));
  EXPECT_EQ(output_waypoint_statuses.size(), 4u);
  EXPECT_EQ(output_waypoint_statuses[0].waypoint_status, nav2_msgs::msg::WaypointStatus::PENDING);
  EXPECT_EQ(output_waypoint_statuses[1].waypoint_status, nav2_msgs::msg::WaypointStatus::PENDING);
  EXPECT_EQ(output_waypoint_statuses[2].waypoint_status, nav2_msgs::msg::WaypointStatus::PENDING);
  EXPECT_EQ(output_waypoint_statuses[3].waypoint_status, nav2_msgs::msg::WaypointStatus::SKIPPED);
}

TEST_F(RemoveInCollisionGoalsTestFixture,
  test_tick_remove_in_collision_goals_find_matching_waypoint_fail)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <RemoveInCollisionGoals service_name="/global_costmap/get_cost_global_costmap"
                                  input_goals="{goals}" output_goals="{goals}"
                                  cost_threshold="253"
                                  input_waypoint_statuses="{waypoint_statuses}"
                                  output_waypoint_statuses="{waypoint_statuses}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Goals poses;
  poses.goals.resize(4);
  poses.goals[0].pose.position.x = 0.0;
  poses.goals[0].pose.position.y = 0.0;

  poses.goals[1].pose.position.x = 0.5;
  poses.goals[1].pose.position.y = 0.0;

  poses.goals[2].pose.position.x = 1.0;
  poses.goals[2].pose.position.y = 0.0;

  poses.goals[3].pose.position.x = 2.0;
  poses.goals[3].pose.position.y = 0.0;

  config_->blackboard->set("goals", poses);

  // create waypoint_statuses and set it on blackboard
  std::vector<nav2_msgs::msg::WaypointStatus> waypoint_statuses(poses.goals.size());
  for (size_t i = 0 ; i < waypoint_statuses.size() ; ++i) {
    waypoint_statuses[i].waypoint_pose = poses.goals[i];
    waypoint_statuses[i].waypoint_index = i;
  }
  // inconsistency between waypoint_statuses and poses
  waypoint_statuses[3].waypoint_pose.pose.position.x = 0.0;

  config_->blackboard->set("waypoint_statuses", waypoint_statuses);

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  // check that it failed and returned the original goals
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
  nav_msgs::msg::Goals output_poses;
  EXPECT_TRUE(config_->blackboard->get("goals", output_poses));

  EXPECT_EQ(output_poses.goals.size(), 4u);
  EXPECT_EQ(output_poses.goals[0], poses.goals[0]);
  EXPECT_EQ(output_poses.goals[1], poses.goals[1]);
  EXPECT_EQ(output_poses.goals[2], poses.goals[2]);
  EXPECT_EQ(output_poses.goals[3], poses.goals[3]);
}

TEST_F(RemoveInCollisionGoalsTestFixture, test_tick_remove_in_collision_goals_fail)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <RemoveInCollisionGoals service_name="/local_costmap/get_cost_local_costmap" input_goals="{goals}" output_goals="{goals}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // create new goal and set it on blackboard
  nav_msgs::msg::Goals poses;
  poses.goals.resize(4);
  poses.goals[0].pose.position.x = 0.0;
  poses.goals[0].pose.position.y = 0.0;

  poses.goals[1].pose.position.x = 0.5;
  poses.goals[1].pose.position.y = 0.0;

  poses.goals[2].pose.position.x = 1.0;
  poses.goals[2].pose.position.y = 0.0;

  poses.goals[3].pose.position.x = 2.0;
  poses.goals[3].pose.position.y = 0.0;

  config_->blackboard->set("goals", poses);

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  // check that it failed and returned the original goals
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
  nav_msgs::msg::Goals output_poses;
  EXPECT_TRUE(config_->blackboard->get("goals", output_poses));

  EXPECT_EQ(output_poses.goals.size(), 4u);
  EXPECT_EQ(output_poses.goals[0], poses.goals[0]);
  EXPECT_EQ(output_poses.goals[1], poses.goals[1]);
  EXPECT_EQ(output_poses.goals[2], poses.goals[2]);
  EXPECT_EQ(output_poses.goals[3], poses.goals[3]);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  RemoveInCollisionGoalsTestFixture::success_server_ =
    std::make_shared<RemoveInCollisionGoalsSuccessService>();
  std::thread success_server_thread([]() {
      rclcpp::spin(RemoveInCollisionGoalsTestFixture::success_server_);
    });

  RemoveInCollisionGoalsTestFixture::failure_server_ =
    std::make_shared<RemoveInCollisionGoalsFailureService>();
  std::thread failure_server_thread([]() {
      rclcpp::spin(RemoveInCollisionGoalsTestFixture::failure_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  success_server_thread.join();
  failure_server_thread.join();

  return all_successful;
}
