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

#ifndef ERROR_CODES__SMOOTHER__SMOOTHER_ERROR_PLUGIN_HPP_
#define ERROR_CODES__SMOOTHER__SMOOTHER_ERROR_PLUGIN_HPP_

#include <string>
#include <memory>

#include "nav2_core/smoother.hpp"
#include "nav2_core/smoother_exceptions.hpp"

namespace nav2_system_tests
{

class UnknownErrorSmoother : public nav2_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override {}

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  bool smooth(
    nav_msgs::msg::Path &,
    const rclcpp::Duration &) override
  {
    throw nav2_core::SmootherException("Unknown Smoother exception");
  }
};

class TimeOutErrorSmoother : public UnknownErrorSmoother
{
  bool smooth(
    nav_msgs::msg::Path &,
    const rclcpp::Duration &)
  {
    throw nav2_core::SmootherTimedOut("Smoother timedOut");
  }
};

class SmoothedPathInCollision : public UnknownErrorSmoother
{
  bool smooth(
    nav_msgs::msg::Path &,
    const rclcpp::Duration &)
  {
    throw nav2_core::SmoothedPathInCollision("Smoother path in collision");
  }
};

class FailedToSmoothPath : public UnknownErrorSmoother
{
  bool smooth(
    nav_msgs::msg::Path &,
    const rclcpp::Duration &)
  {
    throw nav2_core::FailedToSmoothPath("Failed to smooth path");
  }
};

class InvalidPath : public UnknownErrorSmoother
{
  bool smooth(
    nav_msgs::msg::Path &,
    const rclcpp::Duration &)
  {
    throw nav2_core::InvalidPath("Invalid path");
  }
};

}  // namespace nav2_system_tests

#endif  // ERROR_CODES__SMOOTHER__SMOOTHER_ERROR_PLUGIN_HPP_
