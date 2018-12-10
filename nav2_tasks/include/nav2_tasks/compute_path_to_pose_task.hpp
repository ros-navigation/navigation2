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

#ifndef NAV2_TASKS__COMPUTE_PATH_TO_POSE_TASK_HPP_
#define NAV2_TASKS__COMPUTE_PATH_TO_POSE_TASK_HPP_

#include "nav2_tasks/task_client.hpp"
#include "nav2_tasks/task_server.hpp"
#include "nav2_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_tasks
{

using ComputePathToPoseCommand = geometry_msgs::msg::PoseStamped;
using ComputePathToPoseResult = nav2_msgs::msg::Path;

using ComputePathToPoseTaskClient =
  TaskClient<ComputePathToPoseCommand, ComputePathToPoseResult>;

using ComputePathToPoseTaskServer =
  TaskServer<ComputePathToPoseCommand, ComputePathToPoseResult>;

template<>
inline const char * getTaskName<ComputePathToPoseCommand, ComputePathToPoseResult>()
{
  return "ComputePathToPoseTask";
}

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__COMPUTE_PATH_TO_POSE_TASK_HPP_
