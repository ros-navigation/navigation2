// Copyright (c) 2025 Polymath Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__NONBLOCKING_SEQUENCE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__NONBLOCKING_SEQUENCE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{
class NonblockingSequence : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::NonblockingSequence
   * @param name Name for the XML tag for this node
   */
  explicit NonblockingSequence(const std::string & name);

  /**
   * @brief A constructor for nav2_behavior_tree::NonblockingSequence
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  NonblockingSequence(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

protected:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
};
}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__NONBLOCKING_SEQUENCE_HPP_
