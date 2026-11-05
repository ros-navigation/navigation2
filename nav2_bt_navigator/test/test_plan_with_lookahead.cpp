// Copyright (c) 2026
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
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

TEST(PlanWithLookahead, branches)
{
  std::ifstream file(TREE_FILE);
  ASSERT_TRUE(file.good());
  const std::string xml((std::istreambuf_iterator<char>(file)), {});
  const auto call_begin = xml.find("<SubTree ID=\"PlanWithLookahead\"");
  const auto call_end = xml.find("/>", call_begin);
  ASSERT_NE(call_begin, std::string::npos);
  ASSERT_NE(call_end, std::string::npos);
  const auto tree_xml =
    std::string(
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">") +
    xml.substr(call_begin, call_end + 2 - call_begin) + "</BehaviorTree></root>";

  BT::BehaviorTreeFactory factory;
  std::vector<std::string> calls;
  bool velocity_ok = true;
  bool path_ok = true;
  bool valid = true;
  bool planner_ok = true;
  double velocity = 0.5;
  double distance = 0.0;
  factory.registerSimpleAction("GetForwardVelocity", [&](BT::TreeNode & node) {
      calls.push_back("velocity");
      if (!velocity_ok) {return BT::NodeStatus::FAILURE;}
      node.setOutput("forward_velocity", velocity);
      return BT::NodeStatus::SUCCESS;
    }, {BT::OutputPort<double>("forward_velocity")});
  factory.registerSimpleAction("GetPoseFromPath", [&](BT::TreeNode & node) {
      calls.push_back("pose");
      if (!path_ok) {return BT::NodeStatus::FAILURE;}
      node.setOutput("pose", 1);
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<int>("path"), BT::InputPort<int>("index"), BT::OutputPort<int>("pose")});
  factory.registerSimpleAction("TruncatePathLocal", [&](BT::TreeNode & node) {
      calls.push_back("truncate");
      distance = node.getInput<double>("distance_forward").value();
      node.setOutput("output_path", 1);
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<int>("input_path"), BT::OutputPort<int>("output_path"),
        BT::InputPort<double>("distance_backward"), BT::InputPort<double>("distance_forward")});
  factory.registerSimpleAction("ValidatePath", [&](BT::TreeNode &) {
      calls.push_back("validate");
      return valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }, {BT::InputPort<int>("path")});
  factory.registerSimpleAction("ComputePathToPose", [&](BT::TreeNode & node) {
      const bool lookahead = static_cast<bool>(node.getInput<int>("start"));
      calls.push_back(lookahead ? "lookahead" : "current");
      if (lookahead && !planner_ok) {return BT::NodeStatus::FAILURE;}
      node.setOutput("path", 2);
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<int>("start"), BT::InputPort<int>("goal"),
        BT::InputPort<std::string>("planner_id"), BT::OutputPort<int>("path"),
        BT::OutputPort<int>("error_code_id"), BT::OutputPort<std::string>("error_msg")});
  factory.registerSimpleAction("ConcatenatePaths", [&](BT::TreeNode & node) {
      calls.push_back("concatenate");
      node.setOutput("output_path", 3);
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<int>("input_path1"), BT::InputPort<int>("input_path2"),
        BT::OutputPort<int>("output_path")});
  auto blackboard = BT::Blackboard::create();
  factory.registerBehaviorTreeFromFile(SUBTREE_FILE);
  auto tree = factory.createTreeFromText(tree_xml, blackboard);
  auto run = [&]() {calls.clear(); return tree.tickOnce();};

  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls,
    (std::vector<std::string>{"velocity", "pose", "truncate", "pose", "validate", "lookahead",
      "concatenate"}));
  EXPECT_DOUBLE_EQ(distance, 0.5);
  EXPECT_EQ(blackboard->get<int>("path"), 3);
  valid = false;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls,
    (std::vector<std::string>{"velocity", "pose", "truncate", "pose", "validate", "current"}));
  valid = true;
  planner_ok = false;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls,
    (std::vector<std::string>{"velocity", "pose", "truncate", "pose", "validate", "lookahead",
      "current"}));
  planner_ok = true;
  path_ok = false;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, (std::vector<std::string>{"velocity", "pose", "current"}));
  velocity_ok = false;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, (std::vector<std::string>{"velocity", "current"}));
  velocity_ok = true;
  velocity = -0.2;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, (std::vector<std::string>{"velocity", "current"}));
}
