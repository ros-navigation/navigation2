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

#ifndef NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_
#define NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_simple_navigator
{

class SimpleNavigator : public nav2_lifecycle::LifecycleNode
{
public:
  SimpleNavigator();
  ~SimpleNavigator();

protected:
  // The lifecycle interface
  nav2_lifecycle::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  using GoalHandle = rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>;
  using ActionServer = nav2_util::SimpleActionServer<nav2_msgs::action::NavigateToPose>;

  // An action server that implements the NavigateToPose action
  std::unique_ptr<ActionServer> action_server_;

  // The action server callback
  void navigateToPose(const std::shared_ptr<GoalHandle> goal_handle);

  // The SimpleNavigator uses planner and controller actions to carry out its own action
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr planner_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr controller_client_;

  // A regular, non-spinning ROS node that we can use for calls to the action clients
  rclcpp::Node::SharedPtr client_node_;

  // A subscription and callback to handle the topic-based goal published from rviz
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // A client that we'll use to send a command message to our own task server
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr self_client_;
};

}  // namespace nav2_simple_navigator

#endif  // NAV2_SIMPLE_NAVIGATOR__SIMPLE_NAVIGATOR_HPP_
