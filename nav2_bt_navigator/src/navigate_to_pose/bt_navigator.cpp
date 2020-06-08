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

#include "nav2_bt_navigator/navigate_to_pose/bt_navigator.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_bt_navigator/ros_topic_logger.hpp"

namespace nav2_bt_navigator
{

nav2_util::CallbackReturn
NavigateToPoseBtNavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  auto callback_return = BtNavigatorBase::on_configure(state);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose", std::bind(&NavigateToPoseBtNavigator::actionCallback, this), false);

  self_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "navigate_to_pose");

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPoseBtNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  return callback_return;
}

nav2_util::CallbackReturn
NavigateToPoseBtNavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  action_server_->activate();
  return BtNavigatorBase::on_activate(state);
}

nav2_util::CallbackReturn
NavigateToPoseBtNavigator::on_deactivate(const rclcpp_lifecycle::State & state)
{
  action_server_->deactivate();
  return BtNavigatorBase::on_deactivate(state);
}

nav2_util::CallbackReturn
NavigateToPoseBtNavigator::on_cleanup(const rclcpp_lifecycle::State & state)
{
  goal_sub_.reset();
  self_client_.reset();
  action_server_.reset();
  return BtNavigatorBase::on_cleanup(state);
}

void
NavigateToPoseBtNavigator::actionCallback()
{
  initializeGoalPose();

  auto is_canceling = [this]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable. Canceling.");
        return true;
      }

      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server is inactive. Canceling.");
        return true;
      }

      return action_server_->is_cancel_requested();
    };

  RosTopicLogger topic_logger(client_node_, tree_);
  std::shared_ptr<Action::Feedback> feedback_msg = std::make_shared<Action::Feedback>();

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Received goal preemption request");
        action_server_->accept_pending_goal();
        initializeGoalPose();
      }
      topic_logger.flush();

      // action server feedback (pose, duration of task,
      // number of recoveries, and distance remaining to goal)
      nav2_util::getCurrentPose(
        feedback_msg->current_pose, *tf_, global_frame_, robot_frame_, transform_tolerance_);

      geometry_msgs::msg::PoseStamped goal_pose;
      blackboard_->get("goal", goal_pose);

      feedback_msg->distance_remaining = nav2_util::geometry_utils::euclidean_distance(
        feedback_msg->current_pose.pose, goal_pose.pose);

      int recovery_count = 0;
      blackboard_->get<int>("number_recoveries", recovery_count);
      feedback_msg->number_of_recoveries = recovery_count;
      feedback_msg->navigation_time = now() - start_time_;
      action_server_->publish_feedback(feedback_msg);
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);
  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      action_server_->succeeded_current();
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      action_server_->terminate_current();
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      action_server_->terminate_all();
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

void
NavigateToPoseBtNavigator::initializeGoalPose()
{
  auto goal = action_server_->get_current_goal();

  RCLCPP_INFO(
    get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Reset state for new action feedback
  start_time_ = now();
  blackboard_->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard_->set("goal", goal->pose);
}

void
NavigateToPoseBtNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator
