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

/** Log a message through the BT navigator's ROS logger. */
class LogAction : public BT::SyncActionNode
{
public:
  LogAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("level", "Log level: DEBUG, INFO, WARN, ERROR, or FATAL"),
      BT::InputPort<std::string>("message", "Message to log"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__LOG_ACTION_HPP_
