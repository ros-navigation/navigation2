// Copyright (c) 2026 Open Navigation LLC
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

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/log_action.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

class LogActionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("log_action_test");
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", node_);
    factory_.registerNodeType<nav2_behavior_tree::LogAction>("Log");

    original_level_ = rcutils_logging_get_logger_level(node_->get_logger().get_name());
    ASSERT_EQ(
      rcutils_logging_set_logger_level(node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG),
      RCUTILS_RET_OK);
    original_handler_ = rcutils_logging_get_output_handler();
    logs_.clear();
    rcutils_logging_set_output_handler(captureLog);
  }

  void TearDown() override
  {
    rcutils_logging_set_output_handler(original_handler_);
    EXPECT_EQ(
      rcutils_logging_set_logger_level(node_->get_logger().get_name(), original_level_),
      RCUTILS_RET_OK);
  }

  BT::NodeStatus tick(const std::string & attributes)
  {
    const std::string xml =
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\"><Log " + attributes +
      "/></BehaviorTree></root>";
    auto tree = factory_.createTreeFromText(xml, blackboard_);
    logs_.clear();
    return tree.rootNode()->executeTick();
  }

  void expectLog(int severity, const std::string & message)
  {
    ASSERT_EQ(logs_.size(), 1u);
    EXPECT_EQ(logs_.front().first, severity);
    EXPECT_EQ(logs_.front().second, message);
  }

  static void captureLog(
    const rcutils_log_location_t *, int severity, const char * name,
    rcutils_time_point_value_t, const char * format, va_list * args)
  {
    if (std::string(name) != "log_action_test") {
      return;
    }
    char message[1024];
    va_list args_copy;
    va_copy(args_copy, *args);
    std::vsnprintf(message, sizeof(message), format, args_copy);
    va_end(args_copy);
    logs_.emplace_back(severity, message);
  }

  nav2::LifecycleNode::SharedPtr node_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  rcutils_logging_output_handler_t original_handler_{};
  int original_level_{};
  static std::vector<std::pair<int, std::string>> logs_;
};

std::vector<std::pair<int, std::string>> LogActionTest::logs_;

TEST_F(LogActionTest, ValidLevels)
{
  const std::vector<std::pair<std::string, int>> levels = {
    {"DEBUG", RCUTILS_LOG_SEVERITY_DEBUG},
    {"INFO", RCUTILS_LOG_SEVERITY_INFO},
    {"WARN", RCUTILS_LOG_SEVERITY_WARN},
    {"ERROR", RCUTILS_LOG_SEVERITY_ERROR},
    {"FATAL", RCUTILS_LOG_SEVERITY_FATAL},
  };
  for (const auto & level : levels) {
    SCOPED_TRACE(level.first);
    EXPECT_EQ(
      tick("level=\"" + level.first + "\" message=\"Progress: 100% %s\""),
      BT::NodeStatus::SUCCESS);
    expectLog(level.second, "Progress: 100% %s");
  }
}

TEST_F(LogActionTest, MissingInputs)
{
  for (const std::string attributes : {"message=\"Hello\"", "level=\"INFO\"", ""}) {
    SCOPED_TRACE(attributes);
    EXPECT_EQ(tick(attributes), BT::NodeStatus::FAILURE);
    expectLog(RCUTILS_LOG_SEVERITY_ERROR, "Log action requires level and message inputs");
  }
}

TEST_F(LogActionTest, InvalidLevels)
{
  for (const std::string level : {"INVALID", "info", ""}) {
    SCOPED_TRACE(level);
    EXPECT_EQ(
      tick("level=\"" + level + "\" message=\"Hello\""), BT::NodeStatus::FAILURE);
    expectLog(RCUTILS_LOG_SEVERITY_ERROR, "Invalid Log action level: " + level);
  }
}

TEST_F(LogActionTest, BlackboardInputs)
{
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
      <Log level="{level}" message="{message}"/>
    </BehaviorTree></root>)",
    blackboard_);

  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::FAILURE);
  expectLog(RCUTILS_LOG_SEVERITY_ERROR, "Log action requires level and message inputs");

  blackboard_->set("level", std::string("INFO"));
  blackboard_->set("message", std::string("First message"));
  tree.haltTree();
  logs_.clear();
  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  expectLog(RCUTILS_LOG_SEVERITY_INFO, "First message");

  blackboard_->set("level", std::string("WARN"));
  blackboard_->set("message", std::string("Updated message"));
  tree.haltTree();
  logs_.clear();
  EXPECT_EQ(tree.rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  expectLog(RCUTILS_LOG_SEVERITY_WARN, "Updated message");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
