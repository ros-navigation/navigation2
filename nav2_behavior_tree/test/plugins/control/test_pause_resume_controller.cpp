// Copyright (c) 2025 Enjoy Robotics
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
#include "utils/get_node_from_tree.hpp"
#include "nav2_behavior_tree/plugins/control/pause_resume_controller.hpp"
#include "std_srvs/srv/trigger.hpp"

class PauseResumeControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  static void SetUpTestCase()
  {
    nav2_behavior_tree::BehaviorTreeTestFixture::SetUpTestCase();

    ASSERT_TRUE(node_);
    pause_client_ = node_->create_client<std_srvs::srv::Trigger>("pause", true);
    resume_client_ = node_->create_client<std_srvs::srv::Trigger>("resume", true);

    ASSERT_TRUE(factory_);
    factory_->registerNodeType<nav2_behavior_tree::PauseResumeController>("PauseResumeController");
    factory_->registerNodeType<nav2_behavior_tree::DummyNode>("DummyNode");
  }

  static void TearDownTestCase()
  {
    tree_.reset();
    pause_client_.reset();
    resume_client_.reset();
  }

  static void CheckServiceResult(
    const nav2::ServiceClient<std_srvs::srv::Trigger>::SharedPtr & client,
    std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future,
    bool success = true)
  {
    const auto res = client->spin_until_complete(future, std::chrono::seconds(1));
    ASSERT_EQ(res, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    EXPECT_EQ(future.get()->success, success);
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
  static nav2::ServiceClient<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  static nav2::ServiceClient<std_srvs::srv::Trigger>::SharedPtr resume_client_;
};

std::shared_ptr<BT::Tree> PauseResumeControllerTestFixture::tree_ = nullptr;
nav2::ServiceClient<std_srvs::srv::Trigger>::SharedPtr
PauseResumeControllerTestFixture::pause_client_ = nullptr;
nav2::ServiceClient<std_srvs::srv::Trigger>::SharedPtr
PauseResumeControllerTestFixture::resume_client_ = nullptr;

TEST_F(PauseResumeControllerTestFixture, test_incorrect_num_children)
{
  // create tree with incorrect number of children
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PauseResumeController
            pause_service_name="pause"
            resume_service_name="resume">
          </PauseResumeController>
        </BehaviorTree>
      </root>)";
  EXPECT_THROW(
    auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard),
    BT::RuntimeError);
}

TEST_F(PauseResumeControllerTestFixture, test_unused_children)
{
  // create tree with only RESUMED child
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PauseResumeController
            pause_service_name="pause"
            resume_service_name="resume">
            <DummyNode/>  <!-- RESUMED -->
          </PauseResumeController>
        </BehaviorTree>
      </root>)";
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // get pause_resume_controller so we can check state
  auto pause_bt_node =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::PauseResumeController>(tree_);
  ASSERT_NE(pause_bt_node, nullptr);
  using state_t = nav2_behavior_tree::state_t;

  // get dummy nodes so we can change their status
  auto resumed_child =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 0);
  ASSERT_NE(resumed_child, nullptr);
  resumed_child->changeStatus(BT::NodeStatus::RUNNING);

  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);

  // Call pause service, expect RUNNING and PAUSED
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = pause_client_->async_call(req);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::PAUSED);
  CheckServiceResult(pause_client_, future);

  // Tick again, expect RUNNING and PAUSED
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::PAUSED);

  // Call resume service, expect RUNNING and ON_RESUME
  future = resume_client_->async_call(req);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);
  CheckServiceResult(resume_client_, future);

  // Tick again, expect RUNNING and RESUMED
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);
}

TEST_F(PauseResumeControllerTestFixture, test_behavior)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <PauseResumeController
            pause_service_name="pause"
            resume_service_name="resume">
            <DummyNode/>  <!-- RESUMED -->
            <DummyNode/>  <!-- PAUSED -->
            <DummyNode/>  <!-- ON_PAUSE -->
            <DummyNode/>  <!-- ON_RESUME -->
          </PauseResumeController>
        </BehaviorTree>
      </root>)";
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // get pause_resume_controller so we can check state
  auto pause_bt_node =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::PauseResumeController>(tree_);
  ASSERT_NE(pause_bt_node, nullptr);
  using state_t = nav2_behavior_tree::state_t;

  // get dummy nodes so we can change their status
  auto resumed_child =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 0);
  auto paused_child =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 1);
  auto on_pause_child =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 2);
  auto on_resume_child =
    nav2_behavior_tree::get_node_from_tree<nav2_behavior_tree::DummyNode>(tree_, 3);
  ASSERT_NE(resumed_child, nullptr);
  ASSERT_NE(paused_child, nullptr);
  ASSERT_NE(on_pause_child, nullptr);
  ASSERT_NE(on_resume_child, nullptr);

  resumed_child->changeStatus(BT::NodeStatus::RUNNING);
  paused_child->changeStatus(BT::NodeStatus::RUNNING);
  on_pause_child->changeStatus(BT::NodeStatus::RUNNING);
  on_resume_child->changeStatus(BT::NodeStatus::RUNNING);

  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);

  // Call pause service, set ON_PAUSE child to RUNNING, expect RUNNING and ON_PAUSE
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = pause_client_->async_call(req);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::ON_PAUSE);
  CheckServiceResult(pause_client_, future);

  // Change ON_PAUSE child to SUCCESS, expect RUNNING and PAUSED
  on_pause_child->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::PAUSED);

  // Set PAUSED child to SUCCESS, expect RUNNING and PAUSED
  // (should keep ticking unless RESUMED branch succeeds)
  paused_child->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::PAUSED);

  // Set PAUSED child to SKIPPED, expect RUNNING and PAUSED
  paused_child->changeStatus(BT::NodeStatus::SKIPPED);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::PAUSED);

  // Call pause service again, expect RUNNING and PAUSED
  future = pause_client_->async_call(req);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::PAUSED);
  CheckServiceResult(pause_client_, future, false);

  // Call resume service, change ON_RESUME child to FAILURE, expect FAILURE
  future = resume_client_->async_call(req);
  on_resume_child->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);
  CheckServiceResult(resume_client_, future);

  // Halt the tree, expect RUNNING and RESUMED
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);

  // Set resumed child to SUCCESS, expect SUCCESS and RESUMED
  resumed_child->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);

  // Call resume service again, expect RUNNING and RESUMED
  future = resume_client_->async_call(req);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(pause_bt_node->getState(), state_t::RESUMED);
  CheckServiceResult(resume_client_, future, false);
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
