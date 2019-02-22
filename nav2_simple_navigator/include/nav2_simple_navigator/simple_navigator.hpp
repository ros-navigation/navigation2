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

#include <memory>
#include <string>

#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/follow_path_task.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"

namespace nav2_simple_navigator
{

class SimpleNavigator : public nav2_lifecycle::LifecycleNode
{
public:
  SimpleNavigator();
  ~SimpleNavigator();

protected:
  // The lifecycle interface
  nav2_lifecycle::CallbackReturn onConfigure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onActivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onDeactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onCleanup(const rclcpp_lifecycle::State & state) override;

  // The task server receives the NavigateToPose commands, invoking navigateToPose()
  nav2_tasks::TaskStatus navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command);
  std::unique_ptr<nav2_tasks::NavigateToPoseTaskServer> task_server_;

  // The SimpleNavigator uses the planner and controller to carry out the task
  std::unique_ptr<nav2_tasks::ComputePathToPoseTaskClient> planner_client_;
  std::unique_ptr<nav2_tasks::FollowPathTaskClient> controller_client_;
};

}  // namespace nav2_simple_navigator

#endif  // NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_
