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
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"

namespace nav2_bt_navigator
{

bool
NavigateToPoseNavigator::configure(
  nav2::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  goal_blackboard_id_ = node->declare_or_get_parameter(getName() + ".goal_blackboard_id",
      std::string("goal"));
  path_blackboard_id_ = node->declare_or_get_parameter(getName() + ".path_blackboard_id",
      std::string("path"));

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  self_client_ = node->create_action_client<ActionT>(getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  bool enable_groot_monitoring =
    node->declare_or_get_parameter(getName() + ".enable_groot_monitoring", false);
  int groot_server_port =
    node->declare_or_get_parameter(getName() + ".groot_server_port", 1669);

  bt_action_server_->setGrootMonitoring(
      enable_groot_monitoring,
      groot_server_port);

  return true;
}

std::string
NavigateToPoseNavigator::getDefaultBTFilepath(
  nav2::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  std::string pkg_share_dir =
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator");

  auto default_bt_xml_filename = node->declare_or_get_parameter(
    "default_nav_to_pose_bt_xml",
    pkg_share_dir +
    "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");

  return default_bt_xml_filename;
}

bool
NavigateToPoseNavigator::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool
NavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  if (!bt_action_server_->loadBehaviorTree(goal->behavior_tree)) {
    bt_action_server_->setInternalError(ActionT::Result::FAILED_TO_LOAD_BEHAVIOR_TREE,
      std::string("Error loading: ") + goal->behavior_tree + ". Navigation canceled.");
    return false;
  }

  return initializeGoalPose(goal);
}

void
NavigateToPoseNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
  if (result->error_code == 0) {
    if (bt_action_server_->populateInternalError(result)) {
      RCLCPP_WARN(logger_,
        "NavigateToPoseNavigator::goalCompleted, internal error %d:%s.",
        result->error_code,
        result->error_msg.c_str());
    }
  } else {
    RCLCPP_WARN(logger_, "NavigateToPoseNavigator::goalCompleted error %d:%s.",
      result->error_code,
      result->error_msg.c_str());
  }
}

void
NavigateToPoseNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *feedback_utils_.tf,
      feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance))
  {
    RCLCPP_ERROR(logger_, "Robot pose is not available.");
    return;
  }

  auto blackboard = bt_action_server_->getBlackboard();

  // Get current path points
  nav_msgs::msg::Path current_path;
  auto res = blackboard->get(path_blackboard_id_, current_path);
  if (res && current_path.poses.size() > 0u) {
    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    // Calculate distance on the path
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  }

  int recovery_count = 0;
  res = blackboard->get("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateToPoseNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    if (!initializeGoalPose(bt_action_server_->acceptPendingGoal())) {
      RCLCPP_WARN(
        logger_,
        "Preemption request was rejected since the goal pose could not be "
        "transformed. For now, continuing to track the last goal until completion.");
      bt_action_server_->terminatePendingGoal();
    }
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

bool
NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *feedback_utils_.tf,
      feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance))
  {
    bt_action_server_->setInternalError(ActionT::Result::TF_ERROR,
      "Initial robot pose is not available.");
    return false;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!nav2_util::transformPoseInTargetFrame(
      goal->pose, goal_pose, *feedback_utils_.tf, feedback_utils_.global_frame,
      feedback_utils_.transform_tolerance))
  {
    bt_action_server_->setInternalError(ActionT::Result::TF_ERROR,
      "Failed to transform a goal pose provided with frame_id '" +
      goal->pose.header.frame_id +
      "' to the global frame '" +
      feedback_utils_.global_frame +
      "'.");
    return false;
  }

  RCLCPP_INFO(
    logger_, "Begin navigating from current location (%.2f, %.2f) to (%.2f, %.2f)",
    current_pose.pose.position.x, current_pose.pose.position.y,
    goal_pose.pose.position.x, goal_pose.pose.position.y);

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set(goal_blackboard_id_, goal_pose);

  return true;
}

void
NavigateToPoseNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  ActionT::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_bt_navigator::NavigateToPoseNavigator,
  nav2_core::NavigatorBase)
