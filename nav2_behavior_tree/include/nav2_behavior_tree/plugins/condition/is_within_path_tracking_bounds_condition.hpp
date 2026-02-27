// Copyright (c) 2025 Berkan Tali
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_PATH_TRACKING_BOUNDS_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_PATH_TRACKING_BOUNDS_CONDITION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the current path tracking error is within specified bounds and FAILURE otherwise
 */
class IsWithinPathTrackingBoundsCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsWithinPathTrackingBoundsCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsWithinPathTrackingBoundsCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsWithinPathTrackingBoundsCondition() = delete;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

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
      BT::InputPort<double>("max_error_left", 0.5,
        "Maximum allowed position tracking error left side"),
      BT::InputPort<double>("max_error_right", 0.5,
        "Maximum allowed position tracking error on the right side"),
      BT::InputPort<double>("max_error_heading", 3.14,
        "Maximum allowed heading tracking error in radians"),
      BT::InputPort<nav2_msgs::msg::TrackingFeedback>("tracking_feedback",
        "Current tracking feedback message")
    };
  }

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("IsWithinPathTrackingBoundsCondition")};

  double max_error_left_;
  double max_error_right_;
  double max_error_heading_;
  nav2_msgs::msg::TrackingFeedback tracking_feedback_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_PATH_TRACKING_BOUNDS_CONDITION_HPP_
