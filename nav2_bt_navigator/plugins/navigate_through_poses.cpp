// Copyright (c) 2021 Samsung Research
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

#include <vector>
#include <string>

#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"

namespace nav2_bt_navigator
{

bool
NavigateThroughPosesNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();
  node->declare_parameter("goals_blackboard_id", std::string("goals"));
  goals_blackboard_id_ = node->get_parameter("goals_blackboard_id").as_string();
  return true;
}

bool
NavigateThroughPosesNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPoses(goal);

  return true;
}

void
NavigateThroughPosesNavigator::goalCompleted(typename ActionT::Result::SharedPtr /*result*/)
{
}

void
NavigateThroughPosesNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  nav2_util::getCurrentPose(
    feedback_msg->current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  auto blackboard = bt_action_server_->getBlackboard();

  std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
  blackboard->get(goals_blackboard_id_, goal_poses);

  feedback_msg->distance_remaining = nav2_util::geometry_utils::euclidean_distance(
    feedback_msg->current_pose.pose, goal_poses.back().pose);

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->navigation_time = clock_->now() - start_time_;
  // feedback_msg->number_of_poses_remaining = //  TODO feedback from compute path to poses.

  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateThroughPosesNavigator::onPreempt()
{
  RCLCPP_INFO(logger_, "Received goal preemption request");
  initializeGoalPoses(bt_action_server_->acceptPendingGoal());
}

void
NavigateThroughPosesNavigator::initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    logger_, "Begin navigating from current location through %li poses to (%.2f, %.2f)",
    goal->poses.size(), goal->poses.back().pose.position.x, goal->poses.back().pose.position.y);

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(goals_blackboard_id_, goal->poses);
}

}  // namespace nav2_bt_navigator
