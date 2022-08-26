// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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
#include <chrono>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_conversions.hpp"

template<typename T>
class TestNode : public BT::SyncActionNode
{
public:
  TestNode(const std::string & name, const BT::NodeConfiguration & config)
  : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<T>("test")
    };
  }
};


TEST(PointPortTest, test_wrong_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <PointPort test="1.0;2.0;3.0;4.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::Point>>("PointPort");
  auto tree = factory.createTreeFromText(xml_txt);

  geometry_msgs::msg::Point value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.x, 0.0);
  EXPECT_EQ(value.y, 0.0);
  EXPECT_EQ(value.z, 0.0);

  xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <PointPort test="1.0;2.0" />
        </BehaviorTree>
      </root>)";

  tree = factory.createTreeFromText(xml_txt);
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.x, 0.0);
  EXPECT_EQ(value.y, 0.0);
  EXPECT_EQ(value.z, 0.0);
}

TEST(PointPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <PointPort test="1.0;2.0;3.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::Point>>("PointPort");
  auto tree = factory.createTreeFromText(xml_txt);

  geometry_msgs::msg::Point value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.x, 1.0);
  EXPECT_EQ(value.y, 2.0);
  EXPECT_EQ(value.z, 3.0);
}

TEST(QuaternionPortTest, test_wrong_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <QuaternionPort test="1.0;2.0;3.0;4.0;5.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::Quaternion>>("QuaternionPort");
  auto tree = factory.createTreeFromText(xml_txt);

  geometry_msgs::msg::Quaternion value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.x, 0.0);
  EXPECT_EQ(value.y, 0.0);
  EXPECT_EQ(value.z, 0.0);
  EXPECT_EQ(value.w, 1.0);

  xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <QuaternionPort test="1.0;2.0;3.0" />
        </BehaviorTree>
      </root>)";

  tree = factory.createTreeFromText(xml_txt);
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.x, 0.0);
  EXPECT_EQ(value.y, 0.0);
  EXPECT_EQ(value.z, 0.0);
  EXPECT_EQ(value.w, 1.0);
}

TEST(QuaternionPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <QuaternionPort test="0.7;0.0;0.0;0.7" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::Quaternion>>("QuaternionPort");
  auto tree = factory.createTreeFromText(xml_txt);

  geometry_msgs::msg::Quaternion value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.x, 0.7);
  EXPECT_EQ(value.y, 0.0);
  EXPECT_EQ(value.z, 0.0);
  EXPECT_EQ(value.w, 0.7);
}

TEST(PoseStampedPortTest, test_wrong_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <PoseStampedPort test="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;8.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::PoseStamped>>("PoseStampedPort");
  auto tree = factory.createTreeFromText(xml_txt);

  geometry_msgs::msg::PoseStamped value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(rclcpp::Time(value.header.stamp).nanoseconds(), 0);
  EXPECT_EQ(value.header.frame_id, "");
  EXPECT_EQ(value.pose.position.x, 0.0);
  EXPECT_EQ(value.pose.position.y, 0.0);
  EXPECT_EQ(value.pose.position.z, 0.0);
  EXPECT_EQ(value.pose.orientation.x, 0.0);
  EXPECT_EQ(value.pose.orientation.y, 0.0);
  EXPECT_EQ(value.pose.orientation.z, 0.0);
  EXPECT_EQ(value.pose.orientation.w, 1.0);

  xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <PoseStampedPort test="0;map;1.0;2.0;3.0;4.0;5.0;6.0" />
        </BehaviorTree>
      </root>)";

  tree = factory.createTreeFromText(xml_txt);
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(rclcpp::Time(value.header.stamp).nanoseconds(), 0);
  EXPECT_EQ(value.header.frame_id, "");
  EXPECT_EQ(value.pose.position.x, 0.0);
  EXPECT_EQ(value.pose.position.y, 0.0);
  EXPECT_EQ(value.pose.position.z, 0.0);
  EXPECT_EQ(value.pose.orientation.x, 0.0);
  EXPECT_EQ(value.pose.orientation.y, 0.0);
  EXPECT_EQ(value.pose.orientation.z, 0.0);
  EXPECT_EQ(value.pose.orientation.w, 1.0);
}

TEST(PoseStampedPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <PoseStampedPort test="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::PoseStamped>>("PoseStampedPort");
  auto tree = factory.createTreeFromText(xml_txt);

  tree = factory.createTreeFromText(xml_txt);
  geometry_msgs::msg::PoseStamped value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(rclcpp::Time(value.header.stamp).nanoseconds(), 0);
  EXPECT_EQ(value.header.frame_id, "map");
  EXPECT_EQ(value.pose.position.x, 1.0);
  EXPECT_EQ(value.pose.position.y, 2.0);
  EXPECT_EQ(value.pose.position.z, 3.0);
  EXPECT_EQ(value.pose.orientation.x, 4.0);
  EXPECT_EQ(value.pose.orientation.y, 5.0);
  EXPECT_EQ(value.pose.orientation.z, 6.0);
  EXPECT_EQ(value.pose.orientation.w, 7.0);
}

TEST(MillisecondsPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <MillisecondsPort test="10000" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<std::chrono::milliseconds>>("MillisecondsPort");
  auto tree = factory.createTreeFromText(xml_txt);

  std::chrono::milliseconds value;
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.count(), 10000);

  xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <MillisecondsPort test="123.4" />
        </BehaviorTree>
      </root>)";

  tree = factory.createTreeFromText(xml_txt);
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.count(), 123);
}
