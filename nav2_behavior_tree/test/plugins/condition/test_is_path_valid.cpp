// Copyright (c) 2021 Joshua Wallace
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
#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "../../test_service.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class IsPathValidService : public TestService<nav2_msgs::srv::IsPathValid>
{
public:
  IsPathValidService()
  : TestService("is_path_valid")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
  {
    (void)request_header;
    (void)request;
    response->is_valid = true;
  }
};

class IsPathValidTestFixture : public ::testing::Test
{
public:
  void SetUp()
  {
    node_ = std::make_shared<rclcpp::Node>("test_is_path_valid_condition");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));
    factory_->registerNodeType<nav2_behavior_tree::IsPathValidCondition>("IsPathValid");
  }

  void TearDown()
  {
    delete config_;
    config_ = nullptr;
    server_.reset();
    node_.reset();
    factory_.reset();
    tree_.reset();
  }

  static std::shared_ptr<IsPathValidService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<IsPathValidService> IsPathValidTestFixture::server_ = nullptr;
rclcpp::Node::SharedPtr IsPathValidTestFixture::node_ = nullptr;
BT::NodeConfiguration * IsPathValidTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsPathValidTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> IsPathValidTestFixture::tree_ = nullptr;

TEST_F(IsPathValidTestFixture, test_behavior)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <IsPathValid/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  IsPathValidTestFixture::server_ = std::make_shared<IsPathValidService>();
  std::thread server_thread([]() {
      rclcpp::spin(IsPathValidTestFixture::server_);
    });

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
