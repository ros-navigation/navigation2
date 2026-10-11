// Copyright (c) 2026 Nisarg Panchal
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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_manager.hpp"

class FakeRecoveryBehavior : public BT::ActionNodeBase
{
public:
  explicit FakeRecoveryBehavior(const std::string & name)
  : BT::ActionNodeBase(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    tick_count++;
    return status_to_return;
  }

  void halt() override
  {
    resetStatus();
  }

  BT::NodeStatus status_to_return{BT::NodeStatus::SUCCESS};
  int tick_count{0};
};

class RecoveryManagerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    config_->blackboard->set<uint16_t>("compute_path_error_code", 0);
    config_->blackboard->set<uint16_t>("follow_path_error_code", 0);
    config_->input_ports["param_namespace"] = testName();
    config_->input_ports["wrap_around"] = "false";
    for (const auto & behavior_name : {"ClearCostmap", "Wait", "BackUp"}) {
      behaviors_[behavior_name] = std::make_shared<FakeRecoveryBehavior>(behavior_name);
    }
    setErrorCodeNamePrefixes({"compute_path", "follow_path"});
  }

  void TearDown() override
  {
    recovery_manager_.reset();
    behaviors_.clear();
  }

  void createRecoveryManager()
  {
    recovery_manager_ =
      std::make_shared<nav2_behavior_tree::RecoveryManager>("recovery_manager", *config_);
    for (const auto & behavior_name : {"ClearCostmap", "Wait", "BackUp"}) {
      recovery_manager_->addChild(behaviors_[behavior_name].get());
    }
  }

  // Declared by the BT action server in a real system
  void setErrorCodeNamePrefixes(const std::vector<std::string> & prefixes)
  {
    if (!node_->has_parameter("error_code_name_prefixes")) {
      // Dynamically typed so that a test can undeclare it
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      node_->declare_parameter(
        "error_code_name_prefixes", rclcpp::ParameterValue(prefixes), descriptor);
    } else {
      node_->set_parameter(rclcpp::Parameter("error_code_name_prefixes", prefixes));
    }
  }

  void setSequence(const std::string & param_name, const std::vector<std::string> & sequence)
  {
    node_->declare_parameter(testName() + "." + param_name, rclcpp::ParameterValue(sequence));
  }

  // This is how an empty list in the parameter file reaches the node
  void setEmptySequence(const std::string & param_name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node_->declare_parameter(testName() + "." + param_name, rclcpp::ParameterValue(), descriptor);
  }

  void setErrorCode(const std::string & blackboard_key, uint16_t error_code)
  {
    config_->blackboard->set<uint16_t>(blackboard_key, error_code);
  }

  // Ticks one whole recovery. The parent RecoveryNode doesn't halt a finished recovery
  BT::NodeStatus runOneRecovery()
  {
    return recovery_manager_->executeTick();
  }

  int tickCount(const std::string & behavior_name)
  {
    return behaviors_[behavior_name]->tick_count;
  }

protected:
  std::string testName()
  {
    return ::testing::UnitTest::GetInstance()->current_test_info()->name();
  }

  std::shared_ptr<nav2_behavior_tree::RecoveryManager> recovery_manager_;
  std::map<std::string, std::shared_ptr<FakeRecoveryBehavior>> behaviors_;
};

TEST_F(RecoveryManagerTestFixture, test_default_sequence)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap", "Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);
  EXPECT_EQ(tickCount("Wait"), 1);
  EXPECT_EQ(tickCount("BackUp"), 0);
}

TEST_F(RecoveryManagerTestFixture, test_error_specific_sequences)
{
  setSequence("compute_path_error_code.default", {"Wait"});
  setSequence("compute_path_error_code.error_specific.205", {"BackUp"});
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  setSequence("follow_path_error_code.error_specific.105", {"BackUp", "ClearCostmap"});
  createRecoveryManager();

  setErrorCode("follow_path_error_code", 105);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("BackUp"), 1);

  // Every error code has its own place in its own sequence
  setErrorCode("follow_path_error_code", 102);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);

  setErrorCode("follow_path_error_code", 105);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);

  // compute_path comes first in error_code_name_prefixes
  setErrorCode("compute_path_error_code", 208);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);

  setErrorCode("compute_path_error_code", 205);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("BackUp"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_no_recovery_possible)
{
  setSequence("follow_path_error_code.error_specific.103", {"none"});
  setEmptySequence("follow_path_error_code.error_specific.102");
  // Error codes are given by code, not by name
  setSequence("follow_path_error_code.error_specific.failed_to_make_progress", {"ClearCostmap"});
  setSequence("follow_path_error_code.error_specific.0", {"ClearCostmap"});
  createRecoveryManager();

  setErrorCode("follow_path_error_code", 103);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  setErrorCode("follow_path_error_code", 102);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  setErrorCode("follow_path_error_code", 0);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);

  // Without a default, an error code with no error specific sequence isn't recovered from
  setErrorCode("follow_path_error_code", 105);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("ClearCostmap") + tickCount("Wait") + tickCount("BackUp"), 0);
}

TEST_F(RecoveryManagerTestFixture, test_failed_behavior_still_has_its_turn)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap", "Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  behaviors_["ClearCostmap"]->status_to_return = BT::NodeStatus::FAILURE;
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);
  EXPECT_EQ(tickCount("Wait"), 0);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);
}

TEST_F(RecoveryManagerTestFixture, test_abort_on_failure)
{
  setSequence("abort_on_failure", {"BackUp"});
  setSequence("follow_path_error_code.default", {"ClearCostmap", "BackUp", "Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  // A failure of a behavior not in the list is ignored
  behaviors_["ClearCostmap"]->status_to_return = BT::NodeStatus::FAILURE;
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);

  // But not of one in the list
  behaviors_["BackUp"]->status_to_return = BT::NodeStatus::FAILURE;
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("BackUp"), 1);
  EXPECT_EQ(tickCount("Wait"), 0);
}

TEST_F(RecoveryManagerTestFixture, test_abort_on_failure_unknown_behavior)
{
  setSequence("abort_on_failure", {"Spin"});
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_THROW(runOneRecovery(), BT::RuntimeError);
}

TEST_F(RecoveryManagerTestFixture, test_running_behavior)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap", "Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  behaviors_["ClearCostmap"]->status_to_return = BT::NodeStatus::RUNNING;
  EXPECT_EQ(recovery_manager_->executeTick(), BT::NodeStatus::RUNNING);

  // A new error code doesn't interrupt the running behavior
  setErrorCode("compute_path_error_code", 208);
  EXPECT_EQ(recovery_manager_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);

  behaviors_["ClearCostmap"]->status_to_return = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(recovery_manager_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 3);
  EXPECT_EQ(behaviors_["ClearCostmap"]->status(), BT::NodeStatus::IDLE);
}

TEST_F(RecoveryManagerTestFixture, test_halt_while_running_resets_sequences)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap", "Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  behaviors_["Wait"]->status_to_return = BT::NodeStatus::RUNNING;
  EXPECT_EQ(recovery_manager_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tickCount("Wait"), 1);

  recovery_manager_->halt();
  EXPECT_EQ(behaviors_["Wait"]->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_wrap_around)
{
  config_->input_ports["wrap_around"] = "true";
  setSequence("follow_path_error_code.default", {"ClearCostmap", "Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  for (int recovery = 0; recovery < 4; ++recovery) {
    EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(tickCount("ClearCostmap"), 2);
  EXPECT_EQ(tickCount("Wait"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_halt_after_navigation_resets_sequences)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);

  // The tree is halted at the end of each navigation, also reaching nodes that aren't running
  recovery_manager_->halt();
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_custom_error_codes)
{
  setErrorCodeNamePrefixes({"my_action"});
  setSequence("my_action_error_code.default", {"Wait"});
  setSequence("my_action_error_code.error_specific.950", {"BackUp"});
  createRecoveryManager();

  setErrorCode("my_action_error_code", 950);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("BackUp"), 1);

  setErrorCode("my_action_error_code", 951);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);
}

TEST_F(RecoveryManagerTestFixture, test_same_error_code_in_different_groups)
{
  // GOAL_REJECTED is 1 for both the planner and the controller
  setSequence("compute_path_error_code.error_specific.1", {"ClearCostmap", "BackUp"});
  setSequence("follow_path_error_code.error_specific.1", {"Wait", "BackUp"});
  createRecoveryManager();

  setErrorCode("compute_path_error_code", 1);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);

  // The controller's sequence starts from its own first behavior
  setErrorCode("compute_path_error_code", 0);
  setErrorCode("follow_path_error_code", 1);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);
  EXPECT_EQ(tickCount("BackUp"), 0);
}

TEST_F(RecoveryManagerTestFixture, test_error_code_without_recovery_is_skipped)
{
  setSequence("compute_path_error_code.error_specific.206", {"none"});
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  createRecoveryManager();

  // compute_path comes first, but nothing is configured for its error code
  setErrorCode("compute_path_error_code", 206);
  setErrorCode("follow_path_error_code", 105);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);

  // Without another error code to fall back to, there is nothing to run
  setErrorCode("follow_path_error_code", 0);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);
}

TEST_F(RecoveryManagerTestFixture, test_unconfigured_error_codes_are_ignored)
{
  // backup comes first, but a failed BackUp recovery must not take over the recovery
  setErrorCodeNamePrefixes({"backup", "follow_path"});
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  createRecoveryManager();

  setErrorCode("backup_error_code", 710);
  setErrorCode("follow_path_error_code", 105);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);

  setErrorCode("follow_path_error_code", 0);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("ClearCostmap") + tickCount("Wait") + tickCount("BackUp"), 1);
}

TEST_F(RecoveryManagerTestFixture, test_missing_error_code_name_prefixes)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  node_->undeclare_parameter("error_code_name_prefixes");
  createRecoveryManager();
  EXPECT_THROW(recovery_manager_->executeTick(), BT::RuntimeError);
}

TEST_F(RecoveryManagerTestFixture, test_unknown_behavior_name)
{
  setSequence("follow_path_error_code.default", {"Spin"});
  createRecoveryManager();
  EXPECT_THROW(recovery_manager_->executeTick(), BT::RuntimeError);
}

TEST_F(RecoveryManagerTestFixture, test_unused_behavior_only_warns)
{
  setSequence("compute_path_error_code.default", {"ClearCostmap"});
  setSequence("follow_path_error_code.default", {"Wait"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("BackUp"), 0);
}

TEST_F(RecoveryManagerTestFixture, test_inside_subtree)
{
  setSequence("follow_path_error_code.error_specific.105", {"Second"});

  const std::string xml_txt =
    R"(
      <root BTCPP_format="4" main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <SubTree ID="Recovery" _autoremap="true"/>
        </BehaviorTree>
        <BehaviorTree ID="Recovery">
          <RecoveryManager param_namespace="test_inside_subtree">
            <AlwaysSuccess name="First"/>
            <AlwaysSuccess name="Second"/>
          </RecoveryManager>
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<nav2_behavior_tree::RecoveryManager>("RecoveryManager");
  setErrorCode("follow_path_error_code", 105);
  auto tree = factory.createTreeFromText(xml_txt, config_->blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
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
