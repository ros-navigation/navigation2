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

#ifndef NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_
#define NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include "nav2_tasks/task_status.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/follow_path_task.hpp"
#include "nav2_robot/robot.hpp"

namespace nav2_simple_navigator
{

class SimpleNavigator : public nav2_tasks::NavigateToPoseTaskServer
{
public:
  SimpleNavigator();
  ~SimpleNavigator();

  nav2_tasks::TaskStatus execute(const nav2_tasks::NavigateToPoseCommand::SharedPtr command);

protected:
  std::unique_ptr<nav2_tasks::ComputePathToPoseTaskClient> plannerTaskClient_;
  std::unique_ptr<nav2_tasks::FollowPathTaskClient> controllerTaskClient_;

  nav2_robot::Robot robot_;
};

}  // namespace nav2_simple_navigator

#endif  // NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_
