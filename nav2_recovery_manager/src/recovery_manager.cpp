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

#include "nav2_recovery_manager/recovery_manager.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_recovery_manager
{
RecoveryManager::RecoveryManager()
{
  RCLCPP_INFO(get_logger(), "Initializing RecoveryManager...");

  buildRecoveryMap();

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  task_server_ = std::make_unique<nav2_tasks::RecoverTaskServer>(temp_node, false),

  task_server_->setExecuteCallback(
  std::bind(&RecoveryManager::recover, this, std::placeholders::_1));

  // Start listening for incoming Recover task requests
  task_server_->startWorkerThread();
}

RecoveryManager::~RecoveryManager()
{
  RCLCPP_INFO(get_logger(), "Shutting down RecoveryManager");
}

void
RecoveryManager::buildRecoveryMap()
{
  // TODO(orduno) parse from xml file to change on runtime
  // TODO(orduno) change to a plugin pattern
  behavior_map_[RecoverCommand::UNKNOWN] = behaviors_for_unknown();
  behavior_map_[RecoverCommand::BASE_STUCK] = behaviors_for_stuck();
}

std::vector<std::unique<RecoveryBehavior>>
RecoveryManager::behaviors_for_unknown()
{
  std::vector<std::unique<RecoveryBehavior>> behaviors;

  behaviors.push_back(std::make_unique<StopMotion>("Stop Motion"));

  return behaviors;
}

std::vector<std::unique<RecoveryBehavior>>
RecoveryManager::behaviors_for_unknown()
{
  std::vector<std::unique<RecoveryBehavior>> behaviors;

  behaviors.push_back(std::make_unique<StopMotion>("Stop Motion"));
  behaviors.push_back(std::make_unique<FullRotation>("Full Rotation"));

  return behaviors;
}

nav2_tasks::TaskStatus
RecoveryManager::recover(const nav2_task::RecoverCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Attempting to recover from: \"%s\"", failureNameToString(command));

  auto recovery_sequence_it = recovery_map_.find(input);

  if (recovery_sequence_it == recovery_map_.end()) {
    RCLCPP_WARN(get_logger(), "Failed to find a sequence of recovery steps for the given failure";
    return TaskStatus::FAILED;
  }

  auto recovery_sequence = recovery_sequence_it->second();

  for (const auto & behavior : recovery_sequence) {
    if (!behavior.run()) {
      RCLCPP_WARN(get_logger(), "Failed to execute behavior: \"%s\"", behaviour.name());
      return TaskStatus::FAILED;
    }
  }

  // Successfully executed the behaviors
  // TODO(orduno) However the issue might not be solved
  //              we need to use the BT condition that detected the failure
  return TaskStatus::SUCCEEDED;
}

std::string RecoveryManager::failureNameToString(RecoverCommand)
{
  switch (RecoverCommand.result) {
    case RecoverCommand::UNKNOWN:
      return "UNKNOWN";

    case RecoverCommand::BASE_STUCK:
      return "BASE_STUCK";

    default:
      throw std::logic_error("Recoverymanager:: invalid failure");
  }
}

}  // namespace nav2_recovery_manager
