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

#include "nav2_mission_executor/mission_executor.hpp"
#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_mission_executor
{

MissionExecutor::MissionExecutor()
: nav2_tasks::ExecuteMissionTaskServer("ExecuteMissionNode")
{
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("move_base_simple/goal",
      std::bind(&MissionExecutor::onGoalPoseReceived, this, std::placeholders::_1));

  plan_pub_ = create_publisher<nav2_msgs::msg::MissionPlan>(
    "ExecuteMissionTask_command");
}

TaskStatus MissionExecutor::execute(const nav2_tasks::ExecuteMissionCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Executing mission plan: %s", command->mission_plan.c_str());

  // Create and run the behavior tree for this mission
  ExecuteMissionBehaviorTree bt(shared_from_this());
  TaskStatus result =
    bt.run(command->mission_plan, std::bind(&MissionExecutor::cancelRequested, this));

  RCLCPP_INFO(get_logger(), "Completed mission execution: %d", result);
  return result;
}

// Besides receving a mission plan on the command topic, another way to initiate the mission
// is to respond to the goal message sent from rviz. In this case, we'll receive the incoming
// message, dynamically create a mission plan with a single NavigateToPose, then publish the plan
void MissionExecutor::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Received goal pose message");

  // Get a reference for convenience
  geometry_msgs::msg::Pose & p = msg->pose;

  // Compose the args for the NavigateToPose action
  std::stringstream args;
  args << "position=\"" <<
    p.position.x << ";" << p.position.y << ";" << p.position.z << "\"" <<
    " orientation=\"" <<
    p.orientation.x << ";" << p.orientation.y << ";" << p.orientation.z << ";" <<
    p.orientation.w << "\"";

  // Put it all together, trying to make the XML somewhat readable here
  std::stringstream command_ss;
  command_ss <<
    R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <NavigateToPose )" << args.str() <<
    R"(/>
    </SequenceStar>
  </BehaviorTree>
</root>)";

  RCLCPP_INFO(get_logger(), "Publishing a mission plan: %s", command_ss.str().c_str());

  // Publish the mission plan (so that our own TaskServer picks it up)
  auto message = nav2_msgs::msg::MissionPlan();
  message.mission_plan = command_ss.str();

  plan_pub_->publish(message);
}

}  // namespace nav2_mission_executor
