// Copyright (c) 2025 Berkan Tali
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
#include <chrono>

#include "nav2_msgs/msg/tracking_feedback.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/is_within_path_tracking_bounds_condition.hpp"

class IsWithinPathTrackingBoundsConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("test_is_within_path_tracking_bounds");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));

    factory_->registerNodeType<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
      "IsWithinPathTrackingBounds");

    tracking_feedback_pub_ = node_->create_publisher<nav2_msgs::msg::TrackingFeedback>(
      "/tracking_feedback",
      rclcpp::SystemDefaultsQoS());
    tracking_feedback_pub_->on_activate();
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    tracking_feedback_pub_.reset();
    node_.reset();
    factory_.reset();
    executor_.reset();
  }

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static nav2::Publisher<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_pub_;
};

nav2::LifecycleNode::SharedPtr IsWithinPathTrackingBoundsConditionTestFixture::node_ = nullptr;
rclcpp::executors::SingleThreadedExecutor::SharedPtr
IsWithinPathTrackingBoundsConditionTestFixture::executor_ = nullptr;
BT::NodeConfiguration * IsWithinPathTrackingBoundsConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsWithinPathTrackingBoundsConditionTestFixture::factory_ =
  nullptr;
nav2::Publisher<nav2_msgs::msg::TrackingFeedback>::SharedPtr
IsWithinPathTrackingBoundsConditionTestFixture::tracking_feedback_pub_ = nullptr;

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_behavior_within_bounds)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_error="1.0"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  // Test case 1: Error is within bounds (should return SUCCESS)
  nav2_msgs::msg::TrackingFeedback tracking_feedback_msg;
  tracking_feedback_msg.tracking_error = 0.5;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Test case 2: Error is exactly at the boundary (should return SUCCESS)
  tracking_feedback_msg.tracking_error = 1.0;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Test case 3: Error exceeds bounds (should return FAILURE)
  tracking_feedback_msg.tracking_error = 1.5;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Test case 4: Zero error (should return SUCCESS)
  tracking_feedback_msg.tracking_error = 0.0;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_behavior_different_max_error)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_error="0.2"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  // Test case 1: Error is within smaller bounds (should return SUCCESS)
  nav2_msgs::msg::TrackingFeedback tracking_feedback_msg;
  tracking_feedback_msg.tracking_error = 0.1;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Test case 2: Error exceeds smaller bounds (should return FAILURE)
  tracking_feedback_msg.tracking_error = 0.3;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_behavior_large_error_values)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_error="5.0"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  // Test case 1: Large error within bounds (should return SUCCESS)
  nav2_msgs::msg::TrackingFeedback tracking_feedback_msg;
  tracking_feedback_msg.tracking_error = 4.9;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Test case 2: Very large error exceeding bounds (should return FAILURE)
  tracking_feedback_msg.tracking_error = 10.0;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsConditionTestFixture, test_behavior_edge_cases)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <IsWithinPathTrackingBounds max_error="0.0"/>
        </BehaviorTree>
      </root>)";

  auto tree = factory_->createTreeFromText(xml_txt, config_->blackboard);

  // Test case 1: Zero max_error with zero tracking error (should return SUCCESS)
  nav2_msgs::msg::TrackingFeedback tracking_feedback_msg;
  tracking_feedback_msg.tracking_error = 0.0;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // Test case 2: Zero max_error with any positive tracking error (should return FAILURE)
  tracking_feedback_msg.tracking_error = 0.001;
  tracking_feedback_pub_->publish(tracking_feedback_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
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
