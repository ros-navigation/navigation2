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

#ifndef NAV2_LIFECYCLE__LIFECYCLE_HPP_
#define NAV2_LIFECYCLE__LIFECYCLE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_lifecycle
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ILifecycle
{
public:
  virtual ~ILifecycle() {}

  // The basic lifecycle interface (minus onError and onShutdown, which are only relevant for nodes)
  virtual CallbackReturn onConfigure(const rclcpp_lifecycle::State & state) = 0;
  virtual CallbackReturn onActivate(const rclcpp_lifecycle::State & state) = 0;
  virtual CallbackReturn onDeactivate(const rclcpp_lifecycle::State & state) = 0;
  virtual CallbackReturn onCleanup(const rclcpp_lifecycle::State & state) = 0;
};

}  // namespace nav2_lifecycle

#endif  // NAV2_LIFECYCLE__LIFECYCLE_HPP_
