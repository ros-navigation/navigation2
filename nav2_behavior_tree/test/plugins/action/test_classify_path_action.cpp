// Copyright (c) 2026 Origin
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
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/classify_path_action.hpp"

using ClassifiedPose = nav2_msgs::msg::ClassifiedPose;

class ClassifyPathActionServer : public TestActionServer<nav2_msgs::action::ClassifyPath>
{
public:
  ClassifyPathActionServer()
  : TestActionServer("classify_path")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::ClassifyPath>> goal_handle)
  override
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::ClassifyPath::Result>();

    if (!getReturnSuccess()) {
      goal_handle->abort(result);
      return;
    }

    // Classify every pose of the goal path as confined space
    for (const auto & pose : goal->path.poses) {
      ClassifiedPose classified_pose;
      classified_pose.pose = pose;
      classified_pose.class_type = ClassifiedPose::CONFINED_SPACE;
      result->classified_path.push_back(classified_pose);
    }
    result->classification_time = rclcpp::Duration::from_seconds(0.5);
    goal_handle->succeed(result);
  }
};

class ClassifyPathActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("classify_path_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::ClassifyPathAction>(
          name, "classify_path", config);
      };

    factory_->registerBuilder<nav2_behavior_tree::ClassifyPathAction>(
      "ClassifyPath", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    action_server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ClassifyPathActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ClassifyPathActionTestFixture::node_ = nullptr;
std::shared_ptr<ClassifyPathActionServer>
ClassifyPathActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * ClassifyPathActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ClassifyPathActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ClassifyPathActionTestFixture::tree_ = nullptr;

TEST_F(ClassifyPathActionTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClassifyPath path="{path}" classified_path="{classified_path}"
                          classification_time="{classification_time}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav_msgs::msg::Path path;
  for (int i = 0; i < 3; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  config_->blackboard->set<nav_msgs::msg::Path>("path", path);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // goal should have reached our server
  EXPECT_EQ(action_server_->getCurrentGoal()->path, path);

  // outputs should carry the server's result
  auto classified_path =
    config_->blackboard->get<std::vector<ClassifiedPose>>("classified_path");
  ASSERT_EQ(classified_path.size(), path.poses.size());
  for (size_t i = 0; i < classified_path.size(); ++i) {
    EXPECT_EQ(classified_path[i].pose, path.poses[i]);
    EXPECT_EQ(classified_path[i].class_type, ClassifiedPose::CONFINED_SPACE);
  }
  EXPECT_DOUBLE_EQ(config_->blackboard->get<double>("classification_time"), 0.5);

  // halt node so another goal can be sent
  tree_->rootNode()->halt();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST_F(ClassifyPathActionTestFixture, test_failure_clears_outputs)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClassifyPath path="{path}" classified_path="{classified_path}"
                          classification_time="{classification_time}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // Stale outputs from an earlier run should be cleared when the goal aborts
  std::vector<ClassifiedPose> stale(2);
  config_->blackboard->set<std::vector<ClassifiedPose>>("classified_path", stale);
  config_->blackboard->set<double>("classification_time", 1.0);

  action_server_->setReturnSuccess(false);
  while (tree_->rootNode()->status() != BT::NodeStatus::FAILURE) {
    tree_->rootNode()->executeTick();
  }
  action_server_->setReturnSuccess(true);

  EXPECT_TRUE(
    config_->blackboard->get<std::vector<ClassifiedPose>>("classified_path").empty());
  EXPECT_DOUBLE_EQ(config_->blackboard->get<double>("classification_time"), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  ClassifyPathActionTestFixture::action_server_ =
    std::make_shared<ClassifyPathActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(ClassifyPathActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
