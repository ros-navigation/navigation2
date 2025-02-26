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

class BTTestEnv
{
public:
  BTTestEnv()
  {
    node_ = std::make_shared<rclcpp::Node>("in_collision_goals_test_fixture");
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

  ~BTTestEnv()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    success_server_.reset();
    factory_.reset();
  }

  std::shared_ptr<RemoveInCollisionGoalsSuccessService> success_server_;
  std::shared_ptr<RemoveInCollisionGoalsFailureService> failure_server_;
  rclcpp::Node::SharedPtr node_;
  BT::NodeConfiguration * config_;
  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
};

BTTestEnv * test_env = nullptr;

TEST(RemoveInCollisionGoalsTest, test_tick_remove_in_collision_goals_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <RemoveInCollisionGoals service_name="/global_costmap/get_cost_global_costmap" input_goals="{goals}" output_goals="{goals}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  BT::Tree tree_ = test_env->factory_->createTreeFromText(xml_txt, test_env->config_->blackboard);

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStampedArray poses;
  poses.poses.resize(4);
  poses.poses[0].pose.position.x = 0.0;
  poses.poses[0].pose.position.y = 0.0;

  poses.poses[1].pose.position.x = 0.5;
  poses.poses[1].pose.position.y = 0.0;

  poses.poses[2].pose.position.x = 1.0;
  poses.poses[2].pose.position.y = 0.0;

  poses.poses[3].pose.position.x = 2.0;
  poses.poses[3].pose.position.y = 0.0;

  test_env->config_->blackboard->set("goals", poses);

  // tick until node is not running
  tree_.rootNode()->executeTick();
  while (tree_.rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_.rootNode()->executeTick();
  }

  EXPECT_EQ(tree_.rootNode()->status(), BT::NodeStatus::SUCCESS);
  // check that it removed the point in range
  geometry_msgs::msg::PoseStampedArray output_poses;
  EXPECT_TRUE(test_env->config_->blackboard->get("goals", output_poses));

  EXPECT_EQ(output_poses.poses.size(), 3u);
  EXPECT_EQ(output_poses.poses[0], poses.poses[0]);
  EXPECT_EQ(output_poses.poses[1], poses.poses[1]);
  EXPECT_EQ(output_poses.poses[2], poses.poses[2]);
}

TEST(RemoveInCollisionGoalsTest, test_tick_remove_in_collision_goals_fail)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <RemoveInCollisionGoals service_name="/local_costmap/get_cost_local_costmap" input_goals="{goals}" output_goals="{goals}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  BT::Tree tree_ = test_env->factory_->createTreeFromText(xml_txt, test_env->config_->blackboard);

  // create new goal and set it on blackboard
  geometry_msgs::msg::PoseStampedArray poses;
  poses.poses.resize(4);
  poses.poses[0].pose.position.x = 0.0;
  poses.poses[0].pose.position.y = 0.0;

  poses.poses[1].pose.position.x = 0.5;
  poses.poses[1].pose.position.y = 0.0;

  poses.poses[2].pose.position.x = 1.0;
  poses.poses[2].pose.position.y = 0.0;

  poses.poses[3].pose.position.x = 2.0;
  poses.poses[3].pose.position.y = 0.0;

  test_env->config_->blackboard->set("goals", poses);

  // tick until node is not running
  tree_.rootNode()->executeTick();
  while (tree_.rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_.rootNode()->executeTick();
  }

  // check that it failed and returned the original goals
  EXPECT_EQ(tree_.rootNode()->status(), BT::NodeStatus::FAILURE);
  geometry_msgs::msg::PoseStampedArray output_poses;
  EXPECT_TRUE(test_env->config_->blackboard->get("goals", output_poses));

  EXPECT_EQ(output_poses.poses.size(), 4u);
  EXPECT_EQ(output_poses.poses[0], poses.poses[0]);
  EXPECT_EQ(output_poses.poses[1], poses.poses[1]);
  EXPECT_EQ(output_poses.poses[2], poses.poses[2]);
  EXPECT_EQ(output_poses.poses[3], poses.poses[3]);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize test environment
  test_env = new BTTestEnv();

  // stop signal
  std::atomic_bool stop_thread(false);

  // initialize service and spin on new thread
  test_env->success_server_ =
    std::make_shared<RemoveInCollisionGoalsSuccessService>();
  std::thread success_server_thread([&stop_thread]() {
      while (!stop_thread.load()) {
        rclcpp::spin_some(test_env->success_server_);
      }
    });

  test_env->failure_server_ =
    std::make_shared<RemoveInCollisionGoalsFailureService>();
  std::thread failure_server_thread([&stop_thread]() {
      while (!stop_thread.load()) {
        rclcpp::spin_some(test_env->failure_server_);
      }
    });

  int all_successful = RUN_ALL_TESTS();

  // stop the threads
  stop_thread.store(true);
  success_server_thread.join();
  failure_server_thread.join();

  // clean the test environment
  delete test_env;

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
