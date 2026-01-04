// Copyright (c) 2024 Jakub Chudziński
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_NEARBY_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_NEARBY_CONDITION_HPP_

#include <string>
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsGoalNearby
 * service returns true and FAILURE otherwise
 */
class IsGoalNearbyCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsGoalNearbyCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsGoalNearbyCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGoalNearbyCondition() = delete;

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
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),     
      BT::InputPort<double>(
        "proximity_threshold", 3.0,
        "Proximity length (m) of the remaining path considered as a nearby"),
    };
  }

private:
        
    /**
     * @brief Checks if the robot is in the goal proximity
     * @param goal_path current planned path to the goal
     * @param prox_thr proximity length (m) of the remaining path considered as a nearby
     * @return whether the robot is in the goal proximity
     */
    bool isRobotInGoalProximity(
        const nav_msgs::msg::Path& goal_path,
        const double& prox_thr);
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_NEARBY_CONDITION_HPP_