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

#ifndef NAV2_MOTION_PRIMITIVES__BACK_UP_HPP_
#define NAV2_MOTION_PRIMITIVES__BACK_UP_HPP_

#include <chrono>
#include <memory>

#include "nav2_motion_primitives/motion_primitive.hpp"
#include "nav2_tasks/back_up_task.hpp"

namespace nav2_motion_primitives
{

class BackUp : public MotionPrimitive<nav2_tasks::BackUpCommand, nav2_tasks::BackUpResult>
{
public:
  explicit BackUp(rclcpp::Node::SharedPtr & node);
  ~BackUp();

  nav2_tasks::TaskStatus onRun(const nav2_tasks::BackUpCommand::SharedPtr command) override;

  nav2_tasks::TaskStatus onCycleUpdate(nav2_tasks::BackUpResult & result) override;

protected:
  double min_linear_vel_;
  double max_linear_vel_;
  double linear_acc_lim_;

  std::chrono::system_clock::time_point start_time_;

  nav2_tasks::TaskStatus timedBackup();

  nav2_tasks::TaskStatus controlledBackup();
};

}  // namespace nav2_motion_primitives

#endif  // NAV2_MOTION_PRIMITIVES__BACK_UP_HPP_
