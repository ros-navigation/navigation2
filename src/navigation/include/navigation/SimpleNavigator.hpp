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

#ifndef NAVIGATION__SIMPLENAVIGATOR_HPP_
#define NAVIGATION__SIMPLENAVIGATOR_HPP_

#include <string>
#include <memory>
#include "navigation/NavigateToPoseTaskServer.hpp"
#include "planning/ComputePathToPoseTaskClient.hpp"
#include "control/FollowPathTaskClient.hpp"

class SimpleNavigator : public NavigateToPoseTaskServer
{
public:
  explicit SimpleNavigator(const std::string & name);
  SimpleNavigator() = delete;
  ~SimpleNavigator();

  TaskStatus executeAsync(const NavigateToPoseCommand::SharedPtr command);

protected:
  std::unique_ptr<ComputePathToPoseTaskClient> planner_;
  std::unique_ptr<FollowPathTaskClient> controller_;

  void printPlan(nav2_msgs::msg::Path& plan);
};

#endif  // NAVIGATION__SIMPLENAVIGATOR_HPP_
