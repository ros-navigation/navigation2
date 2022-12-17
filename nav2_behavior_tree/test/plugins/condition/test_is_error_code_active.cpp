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

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_error_code_active_condition.hpp"
#include "nav2_msgs/action/follow_path.hpp"

class IsErrorCodeActiveFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  using Action = nav2_msgs::action::FollowPath;
  using ActionGoal = Action::Goal;
  void SetUp()
  {
    std::set<int> current_error_codes = {ActionGoal::NONE};
    std::set<int> error_codes_to_check = {ActionGoal::UNKNOWN};
    config_->blackboard->set("current_error_codes", current_error_codes);
    config_->blackboard->set("error_codes_to_check", error_codes_to_check);

    std::string xml_txt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <IsErrorCodeActive current_error_codes="{current_error_codes}" error_codes_to_check="{error_codes_to_check}"/>
        </BehaviorTree>
      </root>)";

    factory_->registerNodeType<nav2_behavior_tree::IsErrorCodeActive>(
      "IsErrorCodeActive");
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> IsErrorCodeActiveFixture::tree_ = nullptr;

TEST_F(IsErrorCodeActiveFixture, test_condition)
{
  std::map<int, BT::NodeStatus> error_to_status_map = {
    {ActionGoal::NONE, BT::NodeStatus::FAILURE},
    {ActionGoal::UNKNOWN, BT::NodeStatus::SUCCESS},
  };

  for (const auto & error_to_status : error_to_status_map) {
    std::set<int> error = {error_to_status.first};
    config_->blackboard->set("current_error_codes", error);
    EXPECT_EQ(tree_->tickRoot(), error_to_status.second);
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
