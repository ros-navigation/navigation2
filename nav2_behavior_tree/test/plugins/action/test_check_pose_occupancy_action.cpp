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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_service.hpp"
#include "nav2_behavior_tree/plugins/action/check_pose_occupancy_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"

class CheckPoseOccupancySuccessService : public TestService<nav2_msgs::srv::GetCosts>
{
public:
  CheckPoseOccupancySuccessService()
  : TestService("/global_costmap/get_cost_global_costmap")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
  {
    (void)request_header;
    (void)request;
    response->costs = {254};
    response->success = true;
  }
};

class CheckPoseOccupancyFailureService : public TestService<nav2_msgs::srv::GetCosts>
{
public:
  CheckPoseOccupancyFailureService()
  : TestService("/local_costmap/get_cost_local_costmap")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
  {
    (void)request_header;
    (void)request;
    response->costs = {255};
    response->success = false;
  }
};

class CheckPoseOccupancyTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("check_pose_occupancy_test_node");
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
        return std::make_unique<nav2_behavior_tree::CheckPoseOccupancy>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::CheckPoseOccupancy>(
      "CheckPoseOccupancy", builder);
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
  static std::shared_ptr<CheckPoseOccupancySuccessService> success_server_;
  static std::shared_ptr<CheckPoseOccupancyFailureService> failure_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr CheckPoseOccupancyTestFixture::node_ = nullptr;

BT::NodeConfiguration * CheckPoseOccupancyTestFixture::config_ = nullptr;
std::shared_ptr<CheckPoseOccupancySuccessService>
CheckPoseOccupancyTestFixture::success_server_ = nullptr;
std::shared_ptr<CheckPoseOccupancyFailureService>
CheckPoseOccupancyTestFixture::failure_server_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> CheckPoseOccupancyTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> CheckPoseOccupancyTestFixture::tree_ = nullptr;

TEST_F(CheckPoseOccupancyTestFixture, test_tick_check_pose_occupancy_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <CheckPoseOccupancy service_name="/global_costmap/get_cost_global_costmap" pose="{pose}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 0.0;

  config_->blackboard->set("pose", pose);

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

TEST_F(CheckPoseOccupancyTestFixture, test_tick_check_pose_occupancy_failure)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <CheckPoseOccupancy service_name="/local_costmap/get_cost_local_costmap" pose="{pose}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 0.0;

  config_->blackboard->set("pose", pose);

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  CheckPoseOccupancyTestFixture::success_server_ =
    std::make_shared<CheckPoseOccupancySuccessService>();
  std::thread success_server_thread([]() {
      rclcpp::spin(CheckPoseOccupancyTestFixture::success_server_);
    });

  CheckPoseOccupancyTestFixture::failure_server_ =
    std::make_shared<CheckPoseOccupancyFailureService>();
  std::thread failure_server_thread([]() {
      rclcpp::spin(CheckPoseOccupancyTestFixture::failure_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  success_server_thread.join();
  failure_server_thread.join();

  return all_successful;
}
