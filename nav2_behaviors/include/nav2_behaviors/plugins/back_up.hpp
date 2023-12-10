// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIORS__PLUGINS__BACK_UP_HPP_
#define NAV2_BEHAVIORS__PLUGINS__BACK_UP_HPP_

#include <memory>

#include "drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"

using BackUpAction = nav2_msgs::action::BackUp;


namespace nav2_behaviors
{
class BackUp : public DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  using BackUpActionGoal = BackUpAction::Goal;
  using BackUpActionResult = BackUpAction::Result;

  ResultStatus onRun(const std::shared_ptr<const BackUpActionGoal> command) override;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__BACK_UP_HPP_
