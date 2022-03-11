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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_EXPIRING_TIMER_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_EXPIRING_TIMER_CONDITION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS every time a specified
 * time period passes and FAILURE otherwise
 */
class PathExpiringTimerCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PathExpiringTimerCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PathExpiringTimerCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PathExpiringTimerCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", 1.0, "Seconds"),
      BT::InputPort<nav_msgs::msg::Path>("path")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  nav_msgs::msg::Path prev_path_;
  double period_;
  bool first_time_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_EXPIRING_TIMER_CONDITION_HPP_
