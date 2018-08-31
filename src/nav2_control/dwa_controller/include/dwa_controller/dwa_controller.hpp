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

#ifndef DWA_CONTROLLER__DWA_CONTROLLER_HPP_
#define DWA_CONTROLLER__DWA_CONTROLLER_HPP_

#include <string>
#include "nav2_tasks/FollowPathTaskServer.hpp"

namespace dwa_controller
{

class DwaController : public nav2_tasks::FollowPathTaskServer
{
public:
  explicit DwaController(const std::string & name);
  DwaController() = delete;
  ~DwaController();

  nav2_tasks::TaskStatus executeAsync(const nav2_tasks::FollowPathCommand::SharedPtr path) override;
};

}  // namespace dwa_controller

#endif  // DWA_CONTROLLER__DWA_CONTROLLER_HPP_
