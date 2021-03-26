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
#include <set>
#include <memory>

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
  node->declare_parameter("path_blackboard_id", std::string("path"));
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();
  return true;
}

std::string
NavigateThroughPosesNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  if (!node->has_parameter("default_nav_through_poses_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    std::string tree_file = pkg_share_dir +
      "/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml";
    node->declare_parameter("default_nav_through_poses_bt_xml", tree_file);
  }
  node->get_parameter("default_nav_through_poses_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
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
  using namespace nav2_util::geometry_utils;  // NOLINT

  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal, etc)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  auto blackboard = bt_action_server_->getBlackboard();

  Goals goal_poses;
  blackboard->get<Goals>(goals_blackboard_id_, goal_poses);

  if (goal_poses.size() == 0) {
    bt_action_server_->publishFeedback(feedback_msg);
    return;
  }

  nav2_util::getCurrentPose(
    feedback_msg->current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  feedback_msg->distance_remaining = euclidean_distance(
    feedback_msg->current_pose.pose, goal_poses.back().pose);

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->navigation_time = clock_->now() - start_time_;
  feedback_msg->number_of_poses_remaining = goal_poses.size();

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
  if (goal->poses.size() > 0) {
    RCLCPP_INFO(
      logger_, "Begin navigating from current location through %li poses to (%.2f, %.2f)",
      goal->poses.size(), goal->poses.back().pose.position.x, goal->poses.back().pose.position.y);
  }

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<Goals>(goals_blackboard_id_, goal->poses);
}

}  // namespace nav2_bt_navigator
