// Copyright (c) 2025 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/get_current_pose_action.hpp"

using namespace std::chrono_literals;

class GetCurrentPoseTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("get_current_pose_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set("node", node_);
    config_->blackboard->set("tf_buffer", tf_buffer_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::GetCurrentPoseAction>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::GetCurrentPoseAction>(
      "GetCurrentPose", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    tf_broadcaster_.reset();
  }

  void SetUp() override
  {
    if (!node_->has_parameter("robot_base_frame")) {
      node_->declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
    }
    if (!node_->has_parameter("global_frame")) {
      node_->declare_parameter("global_frame", rclcpp::ParameterValue("map"));
    }
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;

  static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

rclcpp::Node::SharedPtr GetCurrentPoseTestFixture::node_ = nullptr;
BT::NodeConfiguration * GetCurrentPoseTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GetCurrentPoseTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GetCurrentPoseTestFixture::tree_ = nullptr;
std::shared_ptr<tf2_ros::Buffer> GetCurrentPoseTestFixture::tf_buffer_ = nullptr;
std::shared_ptr<tf2_ros::TransformListener> GetCurrentPoseTestFixture::tf_listener_ = nullptr;
std::shared_ptr<tf2_ros::TransformBroadcaster> GetCurrentPoseTestFixture::tf_broadcaster_ = nullptr;

TEST_F(GetCurrentPoseTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GetCurrentPose current_pose="{current_pose}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;

  tf_broadcaster_->sendTransform(t);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(100ms);

  int ticks = 0;
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS && ticks < 10) {
    rclcpp::spin_some(node_);
    tree_->rootNode()->executeTick();
    std::this_thread::sleep_for(10ms);
    ticks++;
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  geometry_msgs::msg::PoseStamped current_pose;
  EXPECT_TRUE(config_->blackboard->get("current_pose", current_pose));
  EXPECT_DOUBLE_EQ(current_pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(current_pose.pose.position.y, 2.0);
  EXPECT_EQ(current_pose.header.frame_id, "map");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
