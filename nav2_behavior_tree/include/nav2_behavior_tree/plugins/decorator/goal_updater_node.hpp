// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
// Copyright (c) 2024 Angsa Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/poses_stamped.hpp"

#include "behaviortree_cpp/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class GoalUpdater : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalUpdater
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalUpdater(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::InputPort<nav2_msgs::msg::PosesStamped>("input_goals", "Original Goals"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Received Goal by subscription"),
      BT::OutputPort<nav2_msgs::msg::PosesStamped>("output_goals", "Received Goals by subscription")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback function for goals update topic
   * @param msg Shared pointer to vector of geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goals(const nav2_msgs::msg::PosesStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav2_msgs::msg::PosesStamped>::SharedPtr goals_sub_;

  geometry_msgs::msg::PoseStamped last_goal_received_;
  nav2_msgs::msg::PosesStamped last_goals_received_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // mutex
  std::mutex mutex_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_