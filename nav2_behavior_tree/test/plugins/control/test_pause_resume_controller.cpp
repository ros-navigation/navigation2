// Copyright (c) 2025 Intel Corporation
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

#include "utils/test_behavior_tree_fixture.hpp"
#include "utils/test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/control/pause_resume_controller.hpp"
#include "std_srvs/srv/trigger.hpp"

class PauseResumeControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("pause_resume_controller_test_fixture");
    executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cb_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_->add_callback_group(cb_group_, node_->get_node_base_interface());
    pause_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "pause", rclcpp::ServicesQoS(), cb_group_);
    resume_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "resume", rclcpp::ServicesQoS(), cb_group_);

    config_->input_ports["pause_service_name"] = "pause";
    config_->input_ports["resume_service_name"] = "resume";
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);

    bt_node_ = std::make_shared<nav2_behavior_tree::PauseResumeController>(
      "pause_resume_controller", *config_);
    resumed_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    paused_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    on_pause_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    on_resume_child_ = std::make_shared<nav2_behavior_tree::DummyNode>();
    resumed_child_->changeStatus(BT::NodeStatus::SUCCESS);
    paused_child_->changeStatus(BT::NodeStatus::SUCCESS);
    on_pause_child_->changeStatus(BT::NodeStatus::SUCCESS);
    on_resume_child_->changeStatus(BT::NodeStatus::SUCCESS);
    bt_node_->addChild(resumed_child_.get());
    bt_node_->addChild(paused_child_.get());
    bt_node_->addChild(on_pause_child_.get());
    bt_node_->addChild(on_resume_child_.get());
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::PauseResumeController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> paused_child_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> resumed_child_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> on_pause_child_;
  static std::shared_ptr<nav2_behavior_tree::DummyNode> on_resume_child_;
  static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  static rclcpp::CallbackGroup::SharedPtr cb_group_;
  static rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  static rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
};

std::shared_ptr<nav2_behavior_tree::PauseResumeController>
PauseResumeControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
PauseResumeControllerTestFixture::paused_child_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
PauseResumeControllerTestFixture::resumed_child_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
PauseResumeControllerTestFixture::on_pause_child_ = nullptr;
std::shared_ptr<nav2_behavior_tree::DummyNode>
PauseResumeControllerTestFixture::on_resume_child_ = nullptr;
rclcpp::executors::SingleThreadedExecutor::SharedPtr
PauseResumeControllerTestFixture::executor_ = nullptr;
rclcpp::CallbackGroup::SharedPtr
PauseResumeControllerTestFixture::cb_group_ = nullptr;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
PauseResumeControllerTestFixture::pause_client_ = nullptr;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
PauseResumeControllerTestFixture::resume_client_ = nullptr;

TEST_F(PauseResumeControllerTestFixture, test_behavior)
{
  resumed_child_->changeStatus(BT::NodeStatus::SUCCESS);
  paused_child_->changeStatus(BT::NodeStatus::SUCCESS);
  on_pause_child_->changeStatus(BT::NodeStatus::SUCCESS);
  on_resume_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Set on_pause to RUNNING, call pause service, expect RUNNING
  on_pause_child_->changeStatus(BT::NodeStatus::RUNNING);
  auto res = pause_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  executor_->spin_until_future_complete(res, std::chrono::seconds(1));
  ASSERT_EQ(res.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(res.get()->success, true);

  // Change on_pause to SUCCESS, paused child to FAILURE
  // Expect SUCCESS (from on_pause), then FAILURE (from paused child)
  on_pause_child_->changeStatus(BT::NodeStatus::SUCCESS);
  paused_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Set paused to SUCCESS, tick again, expect SUCCESS
  paused_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Set on_resume to SKIPPED, call resume service, expect SUCCESS (treated same as SKIPPED)
  on_resume_child_->changeStatus(BT::NodeStatus::SKIPPED);
  res = resume_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  executor_->spin_until_future_complete(res, std::chrono::seconds(1));
  ASSERT_EQ(res.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(res.get()->success, true);

  // Set resumed to RUNNING, expect RUNNING
  resumed_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  // Change resumed to SUCCESS, expect SUCCESS
  resumed_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
