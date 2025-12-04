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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/condition/are_poses_near_condition.hpp"

class ArePosesNearTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("test_fixture");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();

    config_->blackboard = BT::Blackboard::create();

    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);

    factory_->registerNodeType<nav2_behavior_tree::ArePosesNearCondition>("ArePosesNear");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;

    tf_listener_.reset();
    tf_buffer_.reset();
    node_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    if (!node_->has_parameter("transform_tolerance")) {
      node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 1.0;
    config_->blackboard->set("p1", goal);

    std::string xml_txt =
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ArePosesNear ref_pose="{p1}" target_pose="{p2}" tolerance="0.5"/>
        </BehaviorTree>
      </root>)";

    tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ArePosesNearTestFixture::node_ = nullptr;
std::shared_ptr<tf2_ros::Buffer> ArePosesNearTestFixture::tf_buffer_ = nullptr;
std::shared_ptr<tf2_ros::TransformListener> ArePosesNearTestFixture::tf_listener_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ArePosesNearTestFixture::factory_ = nullptr;
BT::NodeConfiguration * ArePosesNearTestFixture::config_ = nullptr;
std::shared_ptr<BT::Tree> ArePosesNearTestFixture::tree_ = nullptr;

TEST_F(ArePosesNearTestFixture, test_behavior)
{
  geometry_msgs::msg::PoseStamped p2;
  p2.header.stamp = node_->now();
  p2.header.frame_id = "map";
  p2.pose.position.x = 0.0;
  p2.pose.position.y = 0.0;

  config_->blackboard->set("p2", p2);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::FAILURE);

  p2.pose.position.x = 1.0;
  p2.pose.position.y = 1.0;
  config_->blackboard->set("p2", p2);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::SUCCESS);

  p2.pose.position.x = 1.1;
  p2.pose.position.y = 1.1;
  config_->blackboard->set("p2", p2);
  EXPECT_EQ(tree_->tickOnce(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
