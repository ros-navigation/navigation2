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

#ifndef NAV2_TASKS__SPIN_TASK_HPP_
#define NAV2_TASKS__SPIN_TASK_HPP_

#include "nav2_tasks/task_client.hpp"
#include "nav2_tasks/task_server.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

namespace nav2_tasks
{

using SpinCommand = geometry_msgs::msg::QuaternionStamped;
using SpinResult = std_msgs::msg::Empty;

using SpinTaskClient = TaskClient<SpinCommand, SpinResult>;
using SpinTaskServer = TaskServer<SpinCommand, SpinResult>;

template<>
inline const char * getTaskName<SpinCommand, SpinResult>()
{
  return "SpinTask";
}

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__SPIN_TASK_HPP_
