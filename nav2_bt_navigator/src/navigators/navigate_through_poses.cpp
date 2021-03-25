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
  node->declare_parameter("path_blackboard_id", std::string("path"));
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();
  node->declare_parameter("cull_passed_poses", true);
  cull_passed_poses_ = node->get_parameter("cull_passed_poses").as_bool();

  // due to a quirk with how the default BT XML is read in, we must populate
  // the "goal" field specially due to ports that initialized the key, but not
  // the actualy value, in the blackboard. This sets an actual value to attain
  // rather than a null pointer that results in an any::cast() exception as
  // there is no default constructor.
  geometry_msgs::msg::PoseStamped fake_pose;
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<geometry_msgs::msg::PoseStamped>("goal", fake_pose);

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
  using namespace nav2_util::geometry_utils;

  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
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

  // TODO should this all be a BT node next to compute path through poses?
  // Find and remove any via-point poses already passed from replanning
  // We only need to consider the first point(s)
  // and stop at first point that doesn't meet our criteria for removal

  try {
    // A) find closest point on path
    // In try because if Path is not yet set, this blackboard call will throw
    // an exception, letting us know we have no valid path yet to analyze.
    nav_msgs::msg::Path current_path = blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_);
    unsigned int closest_to_robot =
      nav2_util::geometry_utils::min_by(
      current_path.poses.begin(), current_path.poses.end(),
      [&feedback_msg](const geometry_msgs::msg::PoseStamped & ps) {
        return euclidean_distance(feedback_msg->current_pose.pose, ps.pose);
      }) - current_path.poses.begin();

    // B) find the indices of the goals (or their closest points)
    // if smaller, then passed and should be removed
    // if bigger, then we haven't passed it and we can exit
    while (cull_passed_poses_ && goal_poses.size() > 1) {
      unsigned int closest_to_viapoint =
        nav2_util::geometry_utils::min_by(
        current_path.poses.begin(), current_path.poses.end(),
        [&goal_poses](const geometry_msgs::msg::PoseStamped & ps) {
          return euclidean_distance(goal_poses[0].pose, ps.pose);
        }) - current_path.poses.begin();

      if (closest_to_viapoint > closest_to_robot) {
        break;
      }

      goal_poses.erase(goal_poses.begin());
    }

    blackboard->set<Goals>(goals_blackboard_id_, goal_poses);
  } catch (...) {}

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
