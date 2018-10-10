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
//#include "behavior_tree_core/xml_parsing.h"

using namespace std::chrono_literals;

namespace nav2_mission_executor
{

static const std::string xml_text = R"(
 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <SequenceStar name="root">
            <CalculateGoalPose/>
            <PrintGoalPose />
        </SequenceStar>
     </BehaviorTree>

 </root>
 )";
            //<MoveBase  goal="2;4;0" />
            //<MoveBase  goal="${GoalPose}" />

BT::NodeStatus CalculateGoalPose(BT::TreeNode& self)
{
  const double pi = 3.14159265358979323846;

  geometry_msgs::msg::Pose2D mygoal;
  mygoal.x = 1;
  mygoal.y = 2;
  mygoal.theta = pi;

  // RECOMMENDED: check if the blackboard is nullptr
  if (self.blackboard())
  {
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
    if ( blackboard() && blackboard()->get("GoalPose", goal))
    {
      printf("[PrintGoalPose] x=%.f y=%.1f theta=%.2f\n",
             goal.x, goal.y, goal.theta);
      return BT::NodeStatus::SUCCESS;
    } else {
      printf("The blackboard does not contain the key [GoalPose]\n");
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual void halt() override { setStatus(BT::NodeStatus::IDLE); }
};

ExecuteMissionBehaviorTree::ExecuteMissionBehaviorTree(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Create the input and output messages
  navigateToPoseCommand_ = std::make_shared<nav2_tasks::NavigateToPoseCommand>();
  navigateToPoseResult_ = std::make_shared<nav2_tasks::NavigateToPoseResult>();

  // Compose the NavigateToPose message for the Navigation module. Fake out some values
  // for now. The goal pose would actually come from the Mission Plan. Could pass the mission
  // plan in the constructor and then use the values from there to instance each of the nodes.
  navigateToPoseCommand_->pose.position.x = 0;
  navigateToPoseCommand_->pose.position.y = 1;
  navigateToPoseCommand_->pose.position.z = 2;
  navigateToPoseCommand_->pose.orientation.x = 0;
  navigateToPoseCommand_->pose.orientation.y = 1;
  navigateToPoseCommand_->pose.orientation.z = 2;
  navigateToPoseCommand_->pose.orientation.w = 3;

#if 1
  factory_.registerSimpleAction("CalculateGoalPose", CalculateGoalPose);
  factory_.registerNodeType<PrintGoalPose>("PrintGoalPose");
#else
  // Create the nodes of the tree
  root_ = std::make_unique<BT::SequenceNodeWithMemory>("Sequence");

  navigateToPoseAction1_ = std::make_unique<nav2_tasks::NavigateToPoseAction>(node_,
      "NavigateToPoseAction1", navigateToPoseCommand_, navigateToPoseResult_);
  navigateToPoseAction2_ = std::make_unique<nav2_tasks::NavigateToPoseAction>(node_,
      "NavigateToPoseAction2", navigateToPoseCommand_, navigateToPoseResult_);

  // Add the nodes to the tree, creating the tree structure
  root_->addChild(navigateToPoseAction1_.get());
  root_->addChild(navigateToPoseAction2_.get());
#endif
}

ExecuteMissionBehaviorTree::~ExecuteMissionBehaviorTree()
{
  BT::haltAllActions(root_.get());
}

nav2_tasks::TaskStatus
ExecuteMissionBehaviorTree::run(
  std::function<bool()> /*cancelRequested*/, std::chrono::milliseconds loopTimeout)
{
#if 1
  // create a Blackboard from BlackboardLocal (simple, not persistent, local storage)
  auto blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
 // auto tree = BT::buildTreeFromText(factory_, xml_text, blackboard);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  while (result == BT::NodeStatus::RUNNING)
  {
    //result = tree.root_node->executeTick();
    std::this_thread::sleep_for(loopTimeout);
  }

#else
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = root_->status();

  while (rclcpp::ok() &&
    !(result == BT::NodeStatus::SUCCESS || result == BT::NodeStatus::FAILURE))
  {
    result = root_->executeTick();

    // Check if this task server has received a cancel message
    if (cancelRequested()) {
      root_->halt();
      return nav2_tasks::TaskStatus::CANCELED;
    }

    loopRate.sleep();
  }
#endif

  return (result == BT::NodeStatus::SUCCESS) ?
         nav2_tasks::TaskStatus::SUCCEEDED : nav2_tasks::TaskStatus::FAILED;
}

}  // namespace nav2_mission_executor
