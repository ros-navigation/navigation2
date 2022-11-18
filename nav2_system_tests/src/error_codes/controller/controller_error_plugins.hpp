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

#ifndef ERROR_CODES__CONTROLLER__CONTROLLER_ERROR_PLUGINS_HPP_
#define ERROR_CODES__CONTROLLER__CONTROLLER_ERROR_PLUGINS_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace nav2_system_tests
{

class UnknownErrorController : public nav2_core::Controller
{
public:
  UnknownErrorController() = default;
  ~UnknownErrorController() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}

  void cleanup() {}

  void activate() {}

  void deactivate() {}

  void setPlan(const nav_msgs::msg::Path &) {}

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    throw nav2_core::ControllerException("Unknown Error");
  }

  void setSpeedLimit(const double &, const bool &) {}
};

class TFErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    throw nav2_core::ControllerTFError("TF error");
  }
};

class FailedToMakeProgressErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    throw nav2_core::FailedToMakeProgress("Failed to make progress");
  }
};

class PatienceExceededErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    throw nav2_core::PatienceExceeded("Patience exceeded");
  }
};

class InvalidPathErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    throw nav2_core::InvalidPath("Invalid path");
  }
};

class NoValidControlErrorController : public UnknownErrorController
{
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
  {
    throw nav2_core::NoValidControl("No valid control");
  }
};

}  // namespace nav2_system_tests

#endif  // ERROR_CODES__CONTROLLER__CONTROLLER_ERROR_PLUGINS_HPP_
