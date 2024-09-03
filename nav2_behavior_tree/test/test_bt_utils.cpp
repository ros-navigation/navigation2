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
#include <set>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_utils.hpp"

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
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PointPort test="1.0;2.0;3.0;4.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::Point>>("PointPort");
  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PointPort test="1.0;2.0" />
        </BehaviorTree>
      </root>)";

  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);
}

TEST(PointPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
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
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <QuaternionPort test="1.0;2.0;3.0;4.0;5.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::Quaternion>>("QuaternionPort");

  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <QuaternionPort test="1.0;2.0;3.0" />
        </BehaviorTree>
      </root>)";

  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);
}

TEST(QuaternionPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
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
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PoseStampedPort test="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;8.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<geometry_msgs::msg::PoseStamped>>("PoseStampedPort");
  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PoseStampedPort test="0;map;1.0;2.0;3.0;4.0;5.0;6.0" />
        </BehaviorTree>
      </root>)";

  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);
}

TEST(PoseStampedPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
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

TEST(PoseStampedVectorPortTest, test_wrong_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PoseStampedVectorPortTest test="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;0;map;1.0;2.0;3.0;4.0;5.0;6.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<std::vector<geometry_msgs::msg::PoseStamped>>>(
    "PoseStampedVectorPortTest");
  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PoseStampedVectorPortTest test="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;8.0" />
        </BehaviorTree>
      </root>)";

  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);
}

TEST(PoseStampedVectorPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PoseStampedVectorPortTest test="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;0;odom;8.0;9.0;10.0;11.0;12.0;13.0;14.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<std::vector<geometry_msgs::msg::PoseStamped>>>(
    "PoseStampedVectorPortTest");
  auto tree = factory.createTreeFromText(xml_txt);

  tree = factory.createTreeFromText(xml_txt);
  std::vector<geometry_msgs::msg::PoseStamped> values;
  tree.rootNode()->getInput("test", values);
  EXPECT_EQ(rclcpp::Time(values[0].header.stamp).nanoseconds(), 0);
  EXPECT_EQ(values[0].header.frame_id, "map");
  EXPECT_EQ(values[0].pose.position.x, 1.0);
  EXPECT_EQ(values[0].pose.position.y, 2.0);
  EXPECT_EQ(values[0].pose.position.z, 3.0);
  EXPECT_EQ(values[0].pose.orientation.x, 4.0);
  EXPECT_EQ(values[0].pose.orientation.y, 5.0);
  EXPECT_EQ(values[0].pose.orientation.z, 6.0);
  EXPECT_EQ(values[0].pose.orientation.w, 7.0);
  EXPECT_EQ(rclcpp::Time(values[1].header.stamp).nanoseconds(), 0);
  EXPECT_EQ(values[1].header.frame_id, "odom");
  EXPECT_EQ(values[1].pose.position.x, 8.0);
  EXPECT_EQ(values[1].pose.position.y, 9.0);
  EXPECT_EQ(values[1].pose.position.z, 10.0);
  EXPECT_EQ(values[1].pose.orientation.x, 11.0);
  EXPECT_EQ(values[1].pose.orientation.y, 12.0);
  EXPECT_EQ(values[1].pose.orientation.z, 13.0);
  EXPECT_EQ(values[1].pose.orientation.w, 14.0);
}

TEST(PathPortTest, test_wrong_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PathPortTest test="0;map;0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;0;map;8.0;9.0;10.0;11.0;12.0;13.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<nav_msgs::msg::Path>>(
    "PathPortTest");
  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PathPortTest test="0;map;0;map;1.0;2.0;3.0;4.0;5.0;6.0;" />
        </BehaviorTree>
      </root>)";

  EXPECT_THROW(factory.createTreeFromText(xml_txt), std::exception);
}

TEST(PathPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <PathPortTest test="0;map;0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0;0;map;8.0;9.0;10.0;11.0;12.0;13.0;14.0" />
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<nav_msgs::msg::Path>>(
    "PathPortTest");
  auto tree = factory.createTreeFromText(xml_txt);

  tree = factory.createTreeFromText(xml_txt);
  nav_msgs::msg::Path path;
  tree.rootNode()->getInput("test", path);
  EXPECT_EQ(rclcpp::Time(path.header.stamp).nanoseconds(), 0);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(rclcpp::Time(path.poses[0].header.stamp).nanoseconds(), 0);
  EXPECT_EQ(path.poses[0].header.frame_id, "map");
  EXPECT_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(path.poses[0].pose.position.y, 2.0);
  EXPECT_EQ(path.poses[0].pose.position.z, 3.0);
  EXPECT_EQ(path.poses[0].pose.orientation.x, 4.0);
  EXPECT_EQ(path.poses[0].pose.orientation.y, 5.0);
  EXPECT_EQ(path.poses[0].pose.orientation.z, 6.0);
  EXPECT_EQ(path.poses[0].pose.orientation.w, 7.0);
  EXPECT_EQ(rclcpp::Time(path.poses[1].header.stamp).nanoseconds(), 0);
  EXPECT_EQ(path.poses[1].header.frame_id, "map");
  EXPECT_EQ(path.poses[1].pose.position.x, 8.0);
  EXPECT_EQ(path.poses[1].pose.position.y, 9.0);
  EXPECT_EQ(path.poses[1].pose.position.z, 10.0);
  EXPECT_EQ(path.poses[1].pose.orientation.x, 11.0);
  EXPECT_EQ(path.poses[1].pose.orientation.y, 12.0);
  EXPECT_EQ(path.poses[1].pose.orientation.z, 13.0);
  EXPECT_EQ(path.poses[1].pose.orientation.w, 14.0);
}

TEST(MillisecondsPortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
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
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <MillisecondsPort test="123.4" />
        </BehaviorTree>
      </root>)";

  tree = factory.createTreeFromText(xml_txt);
  tree.rootNode()->getInput("test", value);
  EXPECT_EQ(value.count(), 123);
}

TEST(ErrorCodePortTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ErrorCodePort test="100;204;212"/>
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<std::set<int>>>("ErrorCodePort");
  auto tree = factory.createTreeFromText(xml_txt);

  tree = factory.createTreeFromText(xml_txt);
  std::set<int> value;
  tree.rootNode()->getInput("test", value);

  EXPECT_TRUE(value.find(100) != value.end());
  EXPECT_TRUE(value.find(204) != value.end());
  EXPECT_TRUE(value.find(212) != value.end());
}

TEST(deconflictPortAndParamFrameTest, test_correct_syntax)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ParamPort test="1"/>
        </BehaviorTree>
      </root>)";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestNode<int>>("ParamPort");
  auto tree = factory.createTreeFromText(xml_txt);

  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("test_node");
  node->declare_parameter<int>("test", 2);
  node->declare_parameter<int>("test_alternative", 3);

  int value = BT::deconflictPortAndParamFrame<int>(
    node, "test_alternative", tree.rootNode());

  EXPECT_EQ(value, 3);

  value = BT::deconflictPortAndParamFrame<int>(
    node, "test", tree.rootNode());

  EXPECT_EQ(value, 1);
}
