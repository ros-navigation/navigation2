// Copyright (c) 2025 Maurice Alexander Purnawan
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
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "utils/test_service.hpp"
#include "nav2_behavior_tree/plugins/condition/is_within_path_tracking_bounds_condition.hpp"
#include "utils/test_behavior_tree_fixture.hpp"

class IsWithinPathTrackingBoundsTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ =
      std::make_shared<nav2::LifecycleNode>(
      "is_within_path_tracking_bounds_condition_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::IsWithinPathTrackingBoundsCondition>(
      "IsWithinPathTrackingBounds", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr IsWithinPathTrackingBoundsTestFixture::node_ = nullptr;
BT::NodeConfiguration * IsWithinPathTrackingBoundsTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsWithinPathTrackingBoundsTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> IsWithinPathTrackingBoundsTestFixture::tree_ = nullptr;

TEST_F(IsWithinPathTrackingBoundsTestFixture, test_tick_is_within_path_tracking_bounds_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsWithinPathTrackingBounds max_position_error_left="0.5" max_position_error_right="0.5" max_heading_error="3.14" tracking_feedback="{tracking_feedback}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav2_msgs::msg::TrackingFeedback tracking_feedback;
  tracking_feedback.position_tracking_error = 0.25;
  tracking_feedback.heading_tracking_error = 1.0;

  config_->blackboard->set("tracking_feedback", tracking_feedback);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsWithinPathTrackingBoundsTestFixture, test_tick_is_within_path_tracking_bounds_failure)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsWithinPathTrackingBounds max_position_error_left="0.5" max_position_error_right="0.5" max_heading_error="3.14" tracking_feedback="{tracking_feedback}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav2_msgs::msg::TrackingFeedback tracking_feedback;
  tracking_feedback.position_tracking_error = 0.75;  // Exceeds max_position_error_left
  tracking_feedback.heading_tracking_error = 1.0;

  config_->blackboard->set("tracking_feedback", tracking_feedback);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsTestFixture,
  test_tick_is_within_path_tracking_bounds_failure_heading)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsWithinPathTrackingBounds max_position_error_left="0.5" max_position_error_right="0.5" max_heading_error="3.14" tracking_feedback="{tracking_feedback}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav2_msgs::msg::TrackingFeedback tracking_feedback;
  tracking_feedback.position_tracking_error = 0.25;
  tracking_feedback.heading_tracking_error = 4.0;  // Exceeds max_heading_error

  config_->blackboard->set("tracking_feedback", tracking_feedback);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

TEST_F(IsWithinPathTrackingBoundsTestFixture, test_tick_is_within_path_tracking_bounds_asymmetric)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsWithinPathTrackingBounds max_position_error_left="0.5" max_position_error_right="0.2" max_heading_error="3.14" tracking_feedback="{tracking_feedback}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav2_msgs::msg::TrackingFeedback tracking_feedback;
  tracking_feedback.position_tracking_error = -0.15;  // Within right side bound
  tracking_feedback.heading_tracking_error = 1.0;

  config_->blackboard->set("tracking_feedback", tracking_feedback);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
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
