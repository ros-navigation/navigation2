// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2024 Neobotix GmbH
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

#ifndef NAV2_RVIZ_PLUGINS__UTILS_HPP_
#define NAV2_RVIZ_PLUGINS__UTILS_HPP_

#include <QtWidgets>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status.hpp"

namespace nav2_rviz_plugins
{

/**
   * @brief Load the avaialble plugins into the combo box
   * @param node The node to use for loading the plugins
   * @param server_failed if the server failed to load the plugins, false otherwise
   * @param server_name The name of the server to load plugins for
   * @param plugin_type The type of plugin to load
   * @param combo_box The combo box to add the loaded plugins to
   */
void pluginLoader(
  rclcpp::Node::SharedPtr node, bool & server_failed, const std::string & server_name,
  const std::string & plugin_type, QComboBox * combo_box);

// Create label string from goal status msg
QString getGoalStatusLabel(
  std::string title = "Feedback", int8_t status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__UTILS_HPP_
