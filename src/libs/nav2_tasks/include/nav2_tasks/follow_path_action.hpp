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

#ifndef NAV2_TASKS__FOLLOWPATHACTION_HPP_
#define NAV2_TASKS__FOLLOWPATHACTION_HPP_

#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/follow_path_task.hpp"

namespace nav2_tasks
{

using FollowPathAction =
    BtActionNode<FollowPathCommand, FollowPathResult>;

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__FOLLOWPATHACTION_HPP_
