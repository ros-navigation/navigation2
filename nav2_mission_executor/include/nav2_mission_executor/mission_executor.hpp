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

#ifndef NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_
#define NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_

#include <memory>

#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_tasks/execute_mission_task.hpp"

namespace nav2_mission_executor
{

class MissionExecutor : public nav2_lifecycle::LifecycleNode
{
public:
  MissionExecutor();
  ~MissionExecutor();

protected:
  // Implement the lifecycle interface
  nav2_lifecycle::CallbackReturn onConfigure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onActivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onDeactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onCleanup(const rclcpp_lifecycle::State & state) override;

  // The task server receives the ExecuteMission commands, invoking executeMission()
  nav2_tasks::TaskStatus executeMission(const nav2_tasks::ExecuteMissionCommand::SharedPtr command);
  std::unique_ptr<nav2_tasks::ExecuteMissionTaskServer> task_server_;
};

}  // namespace nav2_mission_executor

#endif  // NAV2_MISSION_EXECUTOR__MISSION_EXECUTOR_HPP_
