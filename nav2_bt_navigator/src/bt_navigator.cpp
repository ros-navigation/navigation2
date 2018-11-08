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

#include <string>
#include <memory>
#include <set>
#include <sstream>
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/bt_conversions.hpp"
#include "Blackboard/blackboard_local.h"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_tasks::NavigateToPoseTaskServer("NavigateToPose"),
  robot_(this)
{
}

TaskStatus
BtNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Start navigating to goal (%.2f, %.2f).",
    command->pose.position.x, command->pose.position.y);

  // Get the current pose from the robot
  auto current = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  if (!robot_.getCurrentPose(current)) {
    RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
    return TaskStatus::FAILED;
  }

  // Create the blackboard that will be shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Put together the PathEndPoints message for the ComputePathToPose
  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  endpoints->start = current->pose.pose;
  endpoints->goal = command->pose;
  endpoints->tolerance = 0.3;

  // The path returned from ComputePath and sent to FollowPath
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  // Set the shared data (commands/results)
  blackboard->set<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("endpoints", endpoints);
  blackboard->set<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path", path);

  std::string xml_text =
    R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <ComputePathToPose endpoints="${endpoints}" path="${path}"/>
      <FollowPath path="${path}"/>
    </SequenceStar>
  </BehaviorTree>
</root>)";

  RCLCPP_INFO(get_logger(), "Behavior tree XML: %s", xml_text.c_str());

  // Create and run the behavior tree
  NavigateToPoseBehaviorTree bt(shared_from_this());
  TaskStatus result = bt.run(blackboard, xml_text, std::bind(&BtNavigator::cancelRequested, this));

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
