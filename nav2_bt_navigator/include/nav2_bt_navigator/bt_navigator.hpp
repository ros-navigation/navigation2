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

#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"

namespace nav2_bt_navigator
{

class BtNavigator : public nav2_lifecycle::LifecycleNode
{
public:
  BtNavigator();
  ~BtNavigator();

protected:
  // Implement the lifecycle interface
  nav2_lifecycle::CallbackReturn onConfigure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onActivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onDeactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onCleanup(const rclcpp_lifecycle::State & state) override;

  // The BtNavigator implements the NavigateToPose interface
  std::unique_ptr<nav2_tasks::NavigateToPoseTaskServer> task_server_;
  nav2_tasks::TaskStatus navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command);
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
