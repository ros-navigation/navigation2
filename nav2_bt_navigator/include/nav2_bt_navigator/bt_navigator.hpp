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

#include "behaviortree_cpp/blackboard/blackboard_local.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_bt_navigator
{

class BtNavigator : public nav2_util::LifecycleNode
{
public:
  BtNavigator();
  ~BtNavigator();

protected:
  // The lifecycle node interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::NavigateToPose>;

  // Our action server implements the NavigateToPose action
  std::unique_ptr<ActionServer> action_server_;

  // The action server callback
  void navigateToPose();

  // Goal pose initialization on the blackboard
  void initializeGoalPose();

  // A subscription and callback to handle the topic-based goal published from rviz
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The goal (on the blackboard) to be passed to ComputePath
  std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_;

  // The path (on the blackboard) to be returned from ComputePath and sent to the FollowPath task
  std::shared_ptr<nav2_msgs::msg::Path> path_;

  // The XML string that defines the Behavior Tree to create
  std::string xml_string_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // The complete behavior tree that results from parsing the incoming XML
  std::unique_ptr<BT::Tree> tree_;

  // A client that we'll use to send a command message to our own task server
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr self_client_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
