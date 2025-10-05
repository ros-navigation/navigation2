// Copyright (c) 2025 Maurice Alexander Purnawan
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_HANDLER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_HANDLER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp/action_node.h"

#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The PathHandlerSelector behavior is used to switch the path handler
 * of the controller server. It subscribes to a topic "path_handler_selector"
 * to get the decision about what path_handler must be used. It is usually used before
 * the FollowPath. The selected_path_handler output port is passed to path_handler_id
 * input port of the FollowPath
 * @note This is an Asynchronous node. It will re-initialize when halted.
 */
class PathHandlerSelector : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PathHandlerSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  PathHandlerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_path_handler",
        "the default path_handler to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "path_handler_selector",
        "the input topic name to select the path_handler"),

      BT::OutputPort<std::string>(
        "selected_path_handler",
        "Selected path_handler by subscription")
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
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the path_handler_selector topic
   *
   * @param msg the message with the id of the path_handler_selector
   */
  void callbackPathHandlerSelect(const std_msgs::msg::String::SharedPtr msg);

  nav2::Subscription<std_msgs::msg::String>::SharedPtr path_handler_selector_sub_;

  std::string last_selected_path_handler_;

  nav2::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
  std::chrono::milliseconds bt_loop_duration_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_HANDLER_SELECTOR_NODE_HPP_
