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

#ifndef MISSION_EXECUTION__MISSIONEXECUTOR_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTOR_HPP_

#include <string>
#include <memory>
#include "nav2_tasks/ExecuteMissionTaskServer.hpp"
#include "nav2_tasks/NavigateToPoseTaskClient.hpp"

namespace mission_execution
{

class MissionExecutor : public nav2_tasks::ExecuteMissionTaskServer
{
public:
  explicit MissionExecutor(const std::string & name);
  MissionExecutor() = delete;
  ~MissionExecutor();

  nav2_tasks::TaskStatus executeAsync(
    const nav2_tasks::ExecuteMissionCommand::SharedPtr command) override;

private:
  std::unique_ptr<nav2_tasks::NavigateToPoseTaskClient> navigationTask_;
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__MISSIONEXECUTOR_HPP_
