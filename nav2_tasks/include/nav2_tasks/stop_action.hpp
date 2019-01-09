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

#ifndef NAV2_TASKS__STOP_ACTION_HPP_
#define NAV2_TASKS__STOP_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/stop_task.hpp"

namespace nav2_tasks
{

class StopAction : public BtActionNode<StopCommand, StopResult>
{
public:
  explicit StopAction(const std::string & action_name)
  : BtActionNode<StopCommand, StopResult>(action_name)
  {
  }

  void onInit() override
  {
    // Create the input and output messages
    command_ = std::make_shared<nav2_tasks::StopCommand>();
    result_ = std::make_shared<nav2_tasks::StopResult>();
  }
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__STOP_ACTION_HPP_
