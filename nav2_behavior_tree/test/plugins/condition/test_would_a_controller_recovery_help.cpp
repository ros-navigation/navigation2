// Copyright (c) 2022 Joshua Wallace
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
#include <map>

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/would_a_controller_recovery_help_condition.hpp"
#include "nav2_msgs/action/follow_path.hpp"

class WouldAControllerRecoveryHelpFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  using Action = nav2_msgs::action::FollowPath;
  using ActionResult = Action::Result;
  void SetUp()
  {
    uint16_t error_code = ActionResult::NONE;
    config_->blackboard->set("error_code", error_code);

    std::string xml_txt =
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <WouldAControllerRecoveryHelp error_code="{error_code}"/>
        </BehaviorTree>
      </root>)";

    factory_->registerNodeType<nav2_behavior_tree::WouldAControllerRecoveryHelp>(
      "WouldAControllerRecoveryHelp");
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> WouldAControllerRecoveryHelpFixture::tree_ = nullptr;

TEST_F(WouldAControllerRecoveryHelpFixture, test_condition)
{
  std::map<uint16_t, BT::NodeStatus> error_to_status_map = {
    {ActionResult::NONE, BT::NodeStatus::FAILURE},
    {ActionResult::UNKNOWN, BT::NodeStatus::SUCCESS},
    {ActionResult::INVALID_CONTROLLER, BT::NodeStatus::FAILURE},
    {ActionResult::TF_ERROR, BT::NodeStatus::FAILURE},
    {ActionResult::INVALID_PATH, BT::NodeStatus::FAILURE},
    {ActionResult::PATIENCE_EXCEEDED, BT::NodeStatus::SUCCESS},
    {ActionResult::FAILED_TO_MAKE_PROGRESS, BT::NodeStatus::SUCCESS},
    {ActionResult::NO_VALID_CONTROL, BT::NodeStatus::SUCCESS},
  };

  for (const auto & error_to_status : error_to_status_map) {
    config_->blackboard->set("error_code", error_to_status.first);
    EXPECT_EQ(tree_->tickOnce(), error_to_status.second);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
