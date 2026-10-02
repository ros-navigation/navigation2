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

#include "nav2_util/robot_utils.hpp"
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
    config_->blackboard->set("goal", geometry_msgs::msg::PoseStamped());
    config_->blackboard->set("goals", nav_msgs::msg::Goals());
    config_->input_ports["param_namespace"] = testName();
    config_->input_ports["error_code_names"] = "compute_path_error_code;follow_path_error_code";
    config_->input_ports["reset_distance"] = "0.0";
    config_->input_ports["wrap_around"] = "false";
    config_->input_ports["global_frame"] = "map";
    config_->input_ports["robot_base_frame"] = "base_link";
    for (const auto & behavior_name : {"ClearCostmap", "Wait", "BackUp"}) {
      behaviors_[behavior_name] = std::make_shared<FakeRecoveryBehavior>(behavior_name);
    }
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

  void setErrorName(const std::string & param_name, int64_t error_code)
  {
    node_->declare_parameter(testName() + "." + param_name, rclcpp::ParameterValue(error_code));
  }

  void setErrorCode(const std::string & blackboard_key, uint16_t error_code)
  {
    config_->blackboard->set<uint16_t>(blackboard_key, error_code);
  }

  // Ticks one whole recovery, followed by the halt the parent RecoveryNode would send
  BT::NodeStatus runOneRecovery()
  {
    const BT::NodeStatus status = recovery_manager_->executeTick();
    recovery_manager_->halt();
    return status;
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
  setSequence("compute_path_error_code.error_specific.start_occupied", {"BackUp"});
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  setSequence("follow_path_error_code.error_specific.FAILED_to_make_progress",
    {"BackUp", "ClearCostmap"});
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

  // compute_path_error_code comes first in error_code_names
  setErrorCode("compute_path_error_code", 208);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);

  setErrorCode("compute_path_error_code", 205);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("BackUp"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_no_recovery_possible)
{
  setSequence("follow_path_error_code.error_specific.invalid_path", {"none"});
  setEmptySequence("follow_path_error_code.error_specific.tf_error");
  setSequence("follow_path_error_code.error_specific.not_an_error", {"ClearCostmap"});
  // Errors are given by name, not by code
  setSequence("follow_path_error_code.error_specific.105", {"ClearCostmap"});
  createRecoveryManager();

  setErrorCode("follow_path_error_code", 103);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  setErrorCode("follow_path_error_code", 102);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  setErrorCode("follow_path_error_code", 0);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("ClearCostmap") + tickCount("Wait") + tickCount("BackUp"), 0);

  // Without a default, every child is used in order
  setErrorCode("follow_path_error_code", 105);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);
  EXPECT_EQ(tickCount("Wait"), 1);
  EXPECT_EQ(tickCount("BackUp"), 1);
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

TEST_F(RecoveryManagerTestFixture, test_reset_on_goal_update)
{
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);

  geometry_msgs::msg::PoseStamped new_goal;
  new_goal.pose.position.x = 1.0;
  config_->blackboard->set("goal", new_goal);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_reset_after_robot_moved)
{
  config_->input_ports["reset_distance"] = "0.5";
  setSequence("follow_path_error_code.default", {"ClearCostmap"});
  createRecoveryManager();
  setErrorCode("follow_path_error_code", 105);

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::FAILURE);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.position.x = 1.0;
  robot_pose.pose.orientation.w = 1.0;
  transform_handler_->updateRobotPose(robot_pose.pose);
  while (!nav2_util::getCurrentPose(robot_pose, *transform_handler_->getBuffer()) ||
    robot_pose.pose.position.x < 0.9)
  {
  }

  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);
}

TEST_F(RecoveryManagerTestFixture, test_custom_error_names)
{
  config_->input_ports["error_code_names"] = "first_action_error_code;second_action_error_code";
  setErrorName("first_action_error_code.error_names.MY_FAILURE", 950);
  setErrorName("second_action_error_code.error_names.my_failure", 951);
  setErrorName("second_action_error_code.error_names.too_big", 70000);
  setSequence("first_action_error_code.default", {"ClearCostmap"});
  setSequence("first_action_error_code.error_specific.my_failure", {"BackUp"});
  setSequence("second_action_error_code.default", {"ClearCostmap"});
  setSequence("second_action_error_code.error_specific.my_failure", {"Wait"});
  setSequence("second_action_error_code.error_specific.too_big", {"Wait"});
  createRecoveryManager();

  setErrorCode("first_action_error_code", 950);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("BackUp"), 1);

  // The same name means another code in the other group
  setErrorCode("first_action_error_code", 0);
  setErrorCode("second_action_error_code", 952);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 1);

  setErrorCode("second_action_error_code", 951);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("Wait"), 1);

  // 70000 doesn't fit an error code, so it doesn't wrap around to 4464
  setErrorCode("second_action_error_code", 4464);
  EXPECT_EQ(runOneRecovery(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tickCount("ClearCostmap"), 2);
  EXPECT_EQ(tickCount("Wait"), 1);
}

TEST_F(RecoveryManagerTestFixture, test_unknown_behavior_name)
{
  setSequence("follow_path_error_code.default", {"Spin"});
  createRecoveryManager();
  EXPECT_THROW(recovery_manager_->executeTick(), BT::RuntimeError);
}

TEST_F(RecoveryManagerTestFixture, test_inside_subtree)
{
  setSequence("follow_path_error_code.error_specific.failed_to_make_progress", {"Second"});

  const std::string xml_txt =
    R"(
      <root BTCPP_format="4" main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <SubTree ID="Recovery" _autoremap="true"/>
        </BehaviorTree>
        <BehaviorTree ID="Recovery">
          <RecoveryManager param_namespace="test_inside_subtree" reset_distance="0.0">
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
