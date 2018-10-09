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

#ifndef NAV2_TASKS__TASK_STATUS_HPP_
#define NAV2_TASKS__TASK_STATUS_HPP_

#include "nav2_msgs/msg/task_status.hpp"

namespace nav2_tasks
{

typedef enum
{
  SUCCEEDED = nav2_msgs::msg::TaskStatus::SUCCEEDED,
  FAILED = nav2_msgs::msg::TaskStatus::FAILED,
  RUNNING = nav2_msgs::msg::TaskStatus::RUNNING,
  CANCELED = nav2_msgs::msg::TaskStatus::CANCELED
} TaskStatus;

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__TASK_STATUS_HPP_
