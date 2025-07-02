/******************************************************************************
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_FOLLOWING_PATH_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_FOLLOWING_PATH_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/tracking_error.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief  Behavior-Tree condition that returns SUCCESS while the robot’s
 *         lateral tracking error is within a configurable bound.
 *
 * Ports:
 *   • max_error   (double)   — Allowed deviation in metres (default: 1.0)  
 *   • error_topic (string)   — Topic publishing nav2_msgs/TrackingError
 */
class IsFollowingPathCondition : public BT::ConditionNode
{
public:
  IsFollowingPathCondition(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ~IsFollowingPathCondition() override = default;

  /// Tick: SUCCESS if |error| ≤ max_error_, else FAILURE.
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_error", 1.0,
        "Maximum allowed tracking error"),
      BT::InputPort<std::string>("error_topic", "/tracking_error",
        "nav2_msgs/msg/TrackingError topic")
    };
  }

private:
  void initSubscriber();   // one-shot init on first tick
  void errorCallback(const nav2_msgs::msg::TrackingError::SharedPtr msg);

  // ---------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::TrackingError>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  std::string topic_;
  double      max_error_      {1.0};
  double      last_error_     {std::numeric_limits<double>::infinity()};
  bool        got_msg_        {false};
  bool        sub_ready_      {false};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_FOLLOWING_PATH_CONDITION_HPP_
