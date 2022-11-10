// Copyright (c) 2022. Joshua Wallace
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef NAV2_WS_CONTROLLER_PLUGINS_HPP
#define NAV2_WS_CONTROLLER_PLUGINS_HPP

#include "unknown_error_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace nav2_error_code_test
{
class TFErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
  {
    throw nav2_core::ControllerTFError("TF error");
  }
};

class FailedToMakeProgressErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker)
  {
    throw nav2_core::FailedToMakeProgress("Failed to make progress");
  }
};

class PatienceExceededErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker)
  {
    throw nav2_core::PatienceExceeded("Patience exceeded");
  }
};

class InvalidPathErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker)
  {
    throw nav2_core::InvalidPath("Invalid path");
  }
};

class NoValidControlErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker)
  {
    throw nav2_core::NoValidControl("No valid control");
  }
};

}  // namespace nav2_core

#endif //NAV2_WS_CONTROLLER_PLUGINS_HPP
