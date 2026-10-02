// Copyright (c) 2026 Open Navigation LLC
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

#include <string>

#include "nav2_behavior_tree/plugins/action/log_action.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_behavior_tree
{

BT::NodeStatus LogAction::tick()
{
  const auto level = getInput<std::string>("level");
  const auto message = getInput<std::string>("message");
  const auto node = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");
  const auto logger = node->get_logger();

  if (!level || !message) {
    RCLCPP_ERROR(logger, "Log action requires level and message inputs");
    return BT::NodeStatus::FAILURE;
  }

  if (level.value() == "DEBUG") {
    RCLCPP_DEBUG(logger, "%s", message.value().c_str());
  } else if (level.value() == "INFO") {
    RCLCPP_INFO(logger, "%s", message.value().c_str());
  } else if (level.value() == "WARN") {
    RCLCPP_WARN(logger, "%s", message.value().c_str());
  } else if (level.value() == "ERROR") {
    RCLCPP_ERROR(logger, "%s", message.value().c_str());
  } else if (level.value() == "FATAL") {
    RCLCPP_FATAL(logger, "%s", message.value().c_str());
  } else {
    RCLCPP_ERROR(logger, "Invalid Log action level: %s", level.value().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::LogAction>("Log");
}
