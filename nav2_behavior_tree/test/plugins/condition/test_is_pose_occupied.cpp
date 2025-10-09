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
#include "nav2_behavior_tree/plugins/condition/is_pose_occupied_condition.hpp"
#include "utils/test_behavior_tree_fixture.hpp"


class IsPoseOccupiedSuccessService : public TestService<nav2_msgs::srv::GetCosts>
{
public:
  IsPoseOccupiedSuccessService()
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

class IsPoseOccupiedFailureService : public TestService<nav2_msgs::srv::GetCosts>
{
public:
  IsPoseOccupiedFailureService()
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

class IsPoseOccupiedTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("is_pose_occupied_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::IsPoseOccupiedCondition>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::IsPoseOccupiedCondition>(
      "IsPoseOccupied", builder);
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
  static std::shared_ptr<IsPoseOccupiedSuccessService> success_server_;
  static std::shared_ptr<IsPoseOccupiedFailureService> failure_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr IsPoseOccupiedTestFixture::node_ = nullptr;

BT::NodeConfiguration * IsPoseOccupiedTestFixture::config_ = nullptr;
std::shared_ptr<IsPoseOccupiedSuccessService>
IsPoseOccupiedTestFixture::success_server_ = nullptr;
std::shared_ptr<IsPoseOccupiedFailureService>
IsPoseOccupiedTestFixture::failure_server_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsPoseOccupiedTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> IsPoseOccupiedTestFixture::tree_ = nullptr;

TEST_F(IsPoseOccupiedTestFixture, test_tick_is_pose_occupied_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsPoseOccupied service_name="/global_costmap/get_cost_global_costmap" pose="{pose}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 0.0;

  config_->blackboard->set("pose", pose);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsPoseOccupiedTestFixture, test_tick_is_pose_occupied_fail)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsPoseOccupied service_name="/local_costmap/get_cost_local_costmap" pose="{pose}" cost_threshold="253"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 5.0;
  pose.pose.position.y = 0.0;

  config_->blackboard->set("pose", pose);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  IsPoseOccupiedTestFixture::success_server_ =
    std::make_shared<IsPoseOccupiedSuccessService>();
  std::thread success_server_thread([]() {
      rclcpp::spin(IsPoseOccupiedTestFixture::success_server_);
    });

  IsPoseOccupiedTestFixture::failure_server_ =
    std::make_shared<IsPoseOccupiedFailureService>();
  std::thread failure_server_thread([]() {
      rclcpp::spin(IsPoseOccupiedTestFixture::failure_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  success_server_thread.join();
  failure_server_thread.join();

  return all_successful;
}
