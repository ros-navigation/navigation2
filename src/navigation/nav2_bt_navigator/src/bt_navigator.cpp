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
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_tasks::NavigateToPoseTaskServer("NavigateToPoseNode")
{
  RCLCPP_INFO(get_logger(), "BtNavigator::BtNavigator");
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "BtNavigator::~BtNavigator");
}

TaskStatus
BtNavigator::execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "BtNavigator::execute");

  // TODO(mjeronimo): Can make this a member variable instead
  NavigateToPoseBehaviorTree bt(this);

  TaskStatus result = bt.run(command);
  RCLCPP_INFO(get_logger(), "BtNavigator::executeAsync: completed: %d", result);

  return result;
}

}  // namespace nav2_bt_navigator
