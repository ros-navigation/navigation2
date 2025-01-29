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

#include <chrono>

#include "nav2_rviz_plugins/utils.hpp"

namespace nav2_rviz_plugins
{

void pluginLoader(
  rclcpp::Node::SharedPtr node, bool & server_failed, const std::string & server_name,
  const std::string & plugin_type, QComboBox * combo_box)
{
  // Do not load the plugins if the combo box is already populated
  if (combo_box->count() > 0) {
    return;
  }

  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node, server_name);

  // Wait for the service to be available before calling it
  bool server_unavailable = false;
  while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), "%s service not available", server_name.c_str());
    server_unavailable = true;
    server_failed = true;
    break;
  }

  // Loading the plugins into the combo box
  // If server unavaialble, let the combo box be empty
  if (server_unavailable) {
    return;
  }
  auto parameters = parameter_client->get_parameters({plugin_type});
  auto str_arr = parameters[0].as_string_array();
  combo_box->addItem("Default");
  for (auto str : str_arr) {
    combo_box->addItem(QString::fromStdString(str));
  }
  combo_box->setCurrentText("Default");
}

QString getGoalStatusLabel(std::string title, int8_t status)
{
  std::string status_str;
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      status_str = "<font color=green>active</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      status_str = "<font color=green>reached</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      status_str = "<font color=orange>canceled</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      status_str = "<font color=red>aborted</color>";
      break;

    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      status_str = "unknown";
      break;

    default:
      status_str = "inactive";
      break;
  }
  return QString(
    std::string(
      "<table><tr><td width=150><b>" + title + ":</b></td><td>" +
      status_str + "</td></tr></table>").c_str());
}

}  // namespace nav2_rviz_plugins
