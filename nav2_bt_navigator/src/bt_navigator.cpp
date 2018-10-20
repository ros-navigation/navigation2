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

#include <sstream>
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_tasks::NavigateToPoseTaskServer("NavigateToPoseNode")
{
}

TaskStatus
BtNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Start navigation to goal (%.2f, %.2f).",
    command->pose.position.x, command->pose.position.y);

  // Get a reference for convenience
  geometry_msgs::msg::Pose & p = command->pose;

  // Compose the args for the ComputePathToPose action
  std::stringstream args;
  args << "position=\"" 
       << p.position.x << ";" << p.position.y << ";" << p.position.z  << "\""
       << " orientation=\""
       << p.orientation.x << ";" << p.orientation.y << ";" << p.orientation.z  << ";"
       << p.orientation.w << "\"";

  // Put it all together, trying to make the XML somewhat readable here
  std::stringstream ss;
  ss << R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <SequenceStar name="root">
      <ComputePathToPose )" << args.str() << R"(/>
      <FollowPath />
    </SequenceStar>
  </BehaviorTree>
</root>)";

  RCLCPP_DEBUG(get_logger(), "Behavior tree XML: %s", ss.str());

  // Create and run the behavior tree
  NavigateToPoseBehaviorTree bt(shared_from_this());
  TaskStatus result = bt.run(ss.str(), std::bind(&BtNavigator::cancelRequested, this));

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
