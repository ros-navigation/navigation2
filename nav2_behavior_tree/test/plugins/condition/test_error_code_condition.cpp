// Copyright (c) 2022 Sarthak Mittal
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

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/error_code_condition.hpp"

class ErrorCodeConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->blackboard->set("error_code_id", 101);

    std::string xml_txt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ErrorCodeCheck error_code_id="{error_code_id}" error_codes_to_check="100,101"/>
        </BehaviorTree>
      </root>)";

    factory_->registerNodeType<nav2_behavior_tree::ErrorCodeCondition>("ErrorCodeCheck");
    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown()
  {
    tree_.reset();
  }

protected:
  static std::shared_ptr<BT::Tree> tree_;
};

std::shared_ptr<BT::Tree> ErrorCodeConditionTestFixture::tree_ = nullptr;

TEST_F(ErrorCodeConditionTestFixture, test_behavior)
{
  EXPECT_EQ(tree_->tickRoot(), BT::NodeStatus::SUCCESS);

  config_->blackboard->set("error_code_id", 105);
  EXPECT_EQ(tree_->tickRoot(), BT::NodeStatus::FAILURE);
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
