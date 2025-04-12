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

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
 *       It will re-initialize when halted.
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
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::InputPort<nav_msgs::msg::Goals>("input_goals", "Original Goals"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal",
          "Received Goal by subscription"),
      BT::OutputPort<nav_msgs::msg::Goals>("output_goals",
          "Received Goals by subscription")
    };
  }

private:
  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();
  /**
   * @brief Function to create ROS interfaces
   */
  void createROSInterfaces();
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
   * @param msg Shared pointer to nav_msgs::msg::Goals message
   */
  void callback_updated_goals(const nav_msgs::msg::Goals::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Goals>::SharedPtr goals_sub_;

  geometry_msgs::msg::PoseStamped last_goal_received_;
  bool last_goal_received_set_{false};
  nav_msgs::msg::Goals last_goals_received_;
  bool last_goals_received_set_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::string goal_updater_topic_;
  std::string goals_updater_topic_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
