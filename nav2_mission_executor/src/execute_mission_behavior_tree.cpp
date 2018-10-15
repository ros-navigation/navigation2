// Copyright (c) 2018 Intel Corporation
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

#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

#include <memory>
#include <thread>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "Blackboard/blackboard_local.h"
#include "behavior_tree_core/xml_parsing.h"

using namespace std::chrono_literals;

namespace nav2_mission_executor
{

static const std::string xml_text = R"(
 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <SequenceStar name="root">
            <NavigateToPoseAction goal="${GoalPose}"/>
            <CalculateGoalPose/>
            <PrintGoalPose />
            <NavigateToPoseAction goal="${GoalPose}"/>
        </SequenceStar>
     </BehaviorTree>

 </root>
 )";

BT::NodeStatus CalculateGoalPose(BT::TreeNode& self)
{
  const double pi = 3.14159265358979323846;

  geometry_msgs::msg::Pose2D mygoal;
  mygoal.x = 1;
  mygoal.y = 2;
  mygoal.theta = pi;

  // RECOMMENDED: check if the blackboard is nullptr
  if (self.blackboard()) {
    // store it in the blackboard
    self.blackboard()->set("GoalPose", mygoal);
  }

  printf("[CalculateGoalPose]\n");
  return BT::NodeStatus::SUCCESS;
}

// Read from the blackboard key: [GoalPose]
class PrintGoalPose: public BT::ActionNodeBase
{
public:
  PrintGoalPose(const std::string& name): ActionNodeBase(name) {}

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Pose2D goal;
    // RECOMMENDED: check if the blackboard is empty
    if (blackboard() && blackboard()->get("GoalPose", goal)) {
      printf("[PrintGoalPose] x=%.f y=%.1f theta=%.2f\n", goal.x, goal.y, goal.theta);
    } else {
      printf("The blackboard does not contain the key [GoalPose]\n");
      return BT::NodeStatus::FAILURE;
    }

    rclcpp::Node::SharedPtr node;
    if (blackboard() && blackboard()->get("node", node)) {
      printf("[PrintGoalPose] node: %p\n", (void *) node.get());
    } else {
      printf("The blackboard does not contain the key [GoalPose]\n");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  virtual void halt() override { setStatus(BT::NodeStatus::IDLE); }
};

ExecuteMissionBehaviorTree::ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Create the input and output messages
  navigateToPoseCommand_ = std::make_shared<nav2_tasks::NavigateToPoseCommand>();
  navigateToPoseResult_ = std::make_shared<nav2_tasks::NavigateToPoseResult>();

  factory_.registerSimpleAction("CalculateGoalPose", CalculateGoalPose);
  factory_.registerNodeType<PrintGoalPose>("PrintGoalPose");
  factory_.registerNodeType<nav2_tasks::NavigateToPoseAction>("NavigateToPoseAction");

  printf("[ExecuteMissionBehaviorTree] node: %p\n", (void *) node_.get());

  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  blackboard_->set<std::chrono::milliseconds>("tick_timeout", std::chrono::milliseconds(100));
  blackboard_->set<nav2_tasks::NavigateToPoseCommand::SharedPtr>("command", navigateToPoseCommand_);
  blackboard_->set<nav2_tasks::NavigateToPoseResult::SharedPtr>("result", navigateToPoseResult_);

  // When the tree goes out of scope, all the nodes are destroyed
  tree_ = BT::buildTreeFromText(factory_, xml_text, blackboard_);
}

ExecuteMissionBehaviorTree::~ExecuteMissionBehaviorTree()
{
}

nav2_tasks::TaskStatus
ExecuteMissionBehaviorTree::run(
  std::function<bool()> cancelRequested, std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING)
  {
    result = tree_->root_node->executeTick();

    // Check if this task server has received a cancel message
    if (cancelRequested()) {
      tree_->root_node->halt();
      return nav2_tasks::TaskStatus::CANCELED;
    }

    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ?
         nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_mission_executor
