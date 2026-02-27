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
#include "nav2_behavior_tree/plugins/action/validate_path_action.hpp"
#include "utils/test_behavior_tree_fixture.hpp"

class ValidatePathService : public TestService<nav2_msgs::srv::IsPathValid>
{
public:
  ValidatePathService()
  : TestService("is_path_valid")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
  {
    (void)request_header;
    (void)request;
    response->success = true;
    response->is_valid = true;
  }
};

class ValidatePathTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("validate_path_test_node");
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
        return std::make_unique<nav2_behavior_tree::ValidatePath>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ValidatePath>(
      "ValidatePath", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    success_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }
  static std::shared_ptr<ValidatePathService> success_server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr ValidatePathTestFixture::node_ = nullptr;

BT::NodeConfiguration * ValidatePathTestFixture::config_ = nullptr;
std::shared_ptr<ValidatePathService>
ValidatePathTestFixture::success_server_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ValidatePathTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ValidatePathTestFixture::tree_ = nullptr;

TEST_F(ValidatePathTestFixture, test_behavior)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ValidatePath />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node is not running
  tree_->rootNode()->executeTick();
  while (tree_->rootNode()->status() == BT::NodeStatus::RUNNING) {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  ValidatePathTestFixture::success_server_ =
    std::make_shared<ValidatePathService>();
  std::thread success_server_thread([]() {
      rclcpp::spin(ValidatePathTestFixture::success_server_);
    });
  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  success_server_thread.join();

  return all_successful;
}
