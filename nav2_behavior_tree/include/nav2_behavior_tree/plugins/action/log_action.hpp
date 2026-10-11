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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LOG_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LOG_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::SyncActionNode that logs a message through the BT navigator's ROS logger
 *
 * Usage in XML:
 * @code
 * <Log level="INFO" message="Starting recovery"/>
 * @endcode
 */
class LogAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::LogAction
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  LogAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing the required level and message input ports
   *
   * The level must be DEBUG, INFO, WARN, ERROR, or FATAL. The message is the text to log.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("level", "Log level: DEBUG, INFO, WARN, ERROR, or FATAL"),
      BT::InputPort<std::string>("message", "Message to log"),
    };
  }

  /**
   * @brief Logs the input message at the specified level using the blackboard's node
   * @return BT::NodeStatus::SUCCESS if the message is logged, or BT::NodeStatus::FAILURE
   * if a required input is missing or the log level is invalid
   */
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LOG_ACTION_HPP_
