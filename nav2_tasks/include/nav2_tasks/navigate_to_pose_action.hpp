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

#ifndef NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_
#define NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_

#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"

namespace nav2_tasks
{

#if 1
class NavigateToPoseAction: public BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>
{
public:
  NavigateToPoseAction(const std::string & action_name)
  : BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>(action_name)
  {
    // Retrieve the parameter using getParam()
    //Pose2D goal; 
    //bool goal_passed = getParam<Pose2D>("goal", goal);
  }

private:
};
#else
using NavigateToPoseAction =
  BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>;
#endif

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_
