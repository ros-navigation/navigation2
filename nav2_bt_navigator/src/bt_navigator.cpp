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

#include <memory>
#include <sstream>
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: Node("NavigateToPoseNode")
{
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  robot_ = std::make_unique<nav2_robot::Robot>(temp_node);

  task_server_ = std::make_unique<nav2_tasks::NavigateToPoseTaskServer>(temp_node);
  task_server_->setExecuteCallback(
    std::bind(&BtNavigator::navigateToPose, this, std::placeholders::_1));
}

TaskStatus
BtNavigator::navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Start navigating to goal (%.2f, %.2f).",
    command->pose.position.x, command->pose.position.y);

  // Get the current pose from the robot
  auto current = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  if (!robot_->getCurrentPose(current)) {
    RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
    return TaskStatus::FAILED;
  }

  // Get a reference to the command pose for convenience
  geometry_msgs::msg::Pose & goal = command->pose;

  // Compose the args for the ComputePathToPose action
  std::stringstream args;
  args << "start_position=\"" <<
    current->pose.pose.position.x << ";" << current->pose.pose.position.y << ";" <<
    current->pose.pose.position.z << "\" " <<
    "start_orientation=\"" <<
    current->pose.pose.orientation.x << ";" << current->pose.pose.orientation.y << ";" <<
    current->pose.pose.orientation.z << ";" << current->pose.pose.orientation.w << "\"" <<
    "goal_position=\"" <<
    goal.position.x << ";" << goal.position.y << ";" << goal.position.z << "\" " <<
    "goal_orientation=\"" <<
    goal.orientation.x << ";" << goal.orientation.y << ";" <<
    goal.orientation.z << ";" << goal.orientation.w << "\"";

  // Put it all together, trying to make the XML somewhat readable here
  std::stringstream command_ss;
  command_ss <<
    R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <ComputePathToPose )" << args.str() <<
    R"(/>
      <FollowPath />
    </SequenceStar>
  </BehaviorTree>
</root>)";

  RCLCPP_INFO(get_logger(), "Behavior tree XML: %s", command_ss.str().c_str());

  // Create and run the behavior tree
  NavigateToPoseBehaviorTree bt(shared_from_this());
  TaskStatus result = bt.run(command_ss.str(),
      std::bind(&nav2_tasks::NavigateToPoseTaskServer::cancelRequested, task_server_.get()));

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
