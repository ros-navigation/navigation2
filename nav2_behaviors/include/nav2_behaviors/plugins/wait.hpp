// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_BEHAVIORS__PLUGINS__WAIT_HPP_
#define NAV2_BEHAVIORS__PLUGINS__WAIT_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace nav2_behaviors
{
using WaitAction = nav2_msgs::action::Wait;

/**
 * @class nav2_behaviors::Wait
 * @brief An action server behavior for waiting a fixed duration
 */
class Wait : public TimedBehavior<WaitAction>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::Wait
   */
  Wait();
  ~Wait();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const WaitAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  rclcpp::Time wait_end_;
  WaitAction::Feedback::SharedPtr feedback_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__WAIT_HPP_
