// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_bt_navigator
{
/**
 * @class nav2_bt_navigator::BtNavigator
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtNavigator : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_bt_navigator::BtNavigator class
   */
  BtNavigator();
  /**
   * @brief A destructor for nav2_bt_navigator::BtNavigator class
   */
  ~BtNavigator();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "NavigationToPoses"; subscription to
   * "goal_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in error state
   * @param state Reference to LifeCycle node state
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::NavigateToPoses>;

  // Our action server implements the NavigateToPoses action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief Action server callbacks
   */
  void navigateToPoses();

  /**
   * @brief Goal poses initialization on the blackboard
   */
  void initializeGoalPoses();

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   */
  void onGoalPosesReceived(const nav_msgs::msg::Path::SharedPtr poses);
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goal_sub_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The XML string that defines the Behavior Tree to create
  std::string xml_string_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // The complete behavior tree that results from parsing the incoming XML
  std::unique_ptr<BT::Tree> tree_;

  // A client that we'll use to send a command message to our own task server
  rclcpp_action::Client<nav2_msgs::action::NavigateToPoses>::SharedPtr self_client_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
