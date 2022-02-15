// Copyright (c) 2022 Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__IS_PATH_LONGER_ON_APPROACH_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__IS_PATH_LONGER_ON_APPROACH_HPP_

#include <string>
#include <memory>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child everytime when the length of
 * the new path is smaller than the old one by the length given by the user.
 */
class IsPathLongerOnApproach : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsPathLongerOnApproach
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPathLongerOnApproach(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<double>(
        "prox_leng", 3.0,
        "Integrated path length proximity to the goal pose to apply the behavior (m)"),
      BT::InputPort<double>(
        "length_factor", 2.0,
        "Length factor to check if the path is significantly longer"),
    };
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief Checks if the global path is updated
   * @param new_path new path to the goal
   * @param old_path current path to the goal
   * @return whether the path is updated for the current goal
   */
  bool isPathUpdated(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path);

  /**
   * @brief Checks if the robot is in the goal proximity
   * @param old_path current path to the goal
   * @param prox_leng proximity length from the goal
   * @return whether the robot is in the goal proximity
   */
  bool isRobotInGoalProximity(
    nav_msgs::msg::Path & old_path,
    double & prox_leng);

  /**
   * @brief Checks if the new path is longer
   * @param new_path new path to the goal
   * @param old_path current path to the goal
   * @param length_factor multipler for path length check
   * @return whether the new path is longer
   */
  bool isNewPathLonger(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path,
    double & length_factor);

private:
  nav_msgs::msg::Path new_path_;
  nav_msgs::msg::Path old_path_;
  double prox_leng_ = std::numeric_limits<double>::max();
  double length_factor_ = std::numeric_limits<double>::max();
  rclcpp::Node::SharedPtr node_;
  bool first_time_ = false;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__IS_PATH_LONGER_ON_APPROACH_HPP_
