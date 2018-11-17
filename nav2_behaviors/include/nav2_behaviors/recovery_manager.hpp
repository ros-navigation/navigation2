// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
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

#ifndef NAV2_RECOVERY_MANAGER__RECOVERY_MANAGER_HPP_
#define NAV2_RECOVERY_MANAGER__RECOVERY_MANAGER_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>

#include "nav2_tasks/recover_task.hpp"
#include "nav2_recovery_manager/recovery_behavior_interface.hpp"

namespace nav2_recovery_manager
{

class RecoveryManager : public rclcpp::Node
{
public:
  RecoveryManager();
  ~RecoveryManager();

  nav2_tasks::TaskStatus recover(const nav2_task::RecoverCommand::SharedPtr command);

private:
  std::unique_ptr<nav2_tasks::RecoverTaskServer> task_server_;

  // Association of a failure to a sequence of recovery behaviors
  std::unordered_map<RecoverCommand, std::vector<std::unique<RecoveryBehavior>>> behavior_map_;

  void buildRecoveryMap();

  std::vector<std::unique<RecoveryBehavior>> behaviors_for_unknown();
  std::vector<std::unique<RecoveryBehavior>> behaviors_for_stuck();

  std::string failureToString(RecoverCommand);
};

}  // namespace nav2_recovery_manager

#endif  // NAV2_RECOVERY_MANAGER__RECOVERY_MANAGER_HPP_