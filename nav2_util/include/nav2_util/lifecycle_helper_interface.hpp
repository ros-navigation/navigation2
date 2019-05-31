// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__LIFECYCLE_HELPER_INTERFACE_HPP_
#define NAV2_UTIL__LIFECYCLE_HELPER_INTERFACE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_util
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// A lifecycle interface minus onError and onShutdown, which are only relevant for nodes. This
// has been proposed for rclcpp: https://github.com/ros2/rclcpp/issues/654 and hopefully can be
// added as rclcpp_lifecycle::LifecycleHelperInterface. If not, we can remove this interface
// and just use the whole rclcpp_lifecycle::LifecycleNodeInterface.

class LifecycleHelperInterface
{
public:
  virtual ~LifecycleHelperInterface() {}

  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & state) = 0;
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & state) = 0;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) = 0;
  virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) = 0;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_HELPER_INTERFACE_HPP_
