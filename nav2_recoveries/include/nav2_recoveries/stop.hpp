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

#ifndef NAV2_RECOVERIES__STOP_HPP_
#define NAV2_RECOVERIES__STOP_HPP_

#include <chrono>
#include <memory>

#include "nav2_recoveries/recovery.hpp"
#include "nav2_tasks/stop_task.hpp"
#include "nav2_tasks/follow_path_task.hpp"

namespace nav2_recoveries
{

class Stop : public Recovery<nav2_tasks::StopCommand, nav2_tasks::StopResult>
{
public:
  explicit Stop(rclcpp::Node::SharedPtr & node);
  ~Stop();

  nav2_tasks::TaskStatus onRun(const nav2_tasks::StopCommand::SharedPtr command) override;

  nav2_tasks::TaskStatus onCycleUpdate(nav2_tasks::StopResult & result) override;

  // For stopping the path following controller from sending commands to the robot
  std::unique_ptr<nav2_tasks::FollowPathTaskClient> controller_client_;
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__STOP_HPP_
