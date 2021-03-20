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

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_util::LifecycleNode("bt_navigator", "", false),
  start_time_(0)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node"
  };

  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("odom_topic", std::string("odom"));
  declare_parameter("goal_blackboard_id", std::string("goal"));
}

BtNavigator::~BtNavigator()
{
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  self_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    shared_from_this(), "navigate_to_pose");

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&BtNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  goal_blackboard_id_ = get_parameter("goal_blackboard_id").as_string();

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<Action>>(
    shared_from_this(),
    "navigate_to_pose",
    plugin_lib_names,
    std::bind(&BtNavigator::onGoalReceived, this, std::placeholders::_1),
    std::bind(&BtNavigator::onLoop, this),
    std::bind(&BtNavigator::onPreempt, this));

  if (!bt_action_server_->on_configure()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
  blackboard->set<bool>("initial_pose_received", false);  // NOLINT
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(get_node_base_interface(), 0.3);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!bt_action_server_->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!bt_action_server_->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // TODO(orduno) Fix the race condition between the worker thread ticking the tree
  //              and the main thread resetting the resources, see #1344
  goal_sub_.reset();
  self_client_.reset();

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  if (!bt_action_server_->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  bt_action_server_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  if (!bt_action_server_->on_shutdown()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

bool
BtNavigator::onGoalReceived(Action::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      get_logger(), "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

void
BtNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // time remaining, number of recoveries, and
  // distance remaining to goal)
  auto feedback_msg = std::make_shared<Action::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *tf_, global_frame_, robot_frame_, transform_tolerance_);

  auto blackboard = bt_action_server_->getBlackboard();

  // Get current path points
  nav_msgs::msg::Path current_path;
  blackboard->get("path", current_path);

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
  double current_linear_speed = hypot(current_odom.linear.x, current_odom.linear.y);

  // Calculate estimated time taken to goal if speed is higher than 1cm/s
  // and at least 10cm to go
  if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
    estimated_time_remaining =
      rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->current_pose = current_pose;
  feedback_msg->distance_remaining = distance_remaining;
  feedback_msg->estimated_time_remaining = estimated_time_remaining;
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->navigation_time = now() - start_time_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
BtNavigator::onPreempt()
{
  RCLCPP_INFO(get_logger(), "Received goal preemption request");
  initializeGoalPose(bt_action_server_->acceptPendingGoal());
}

void
BtNavigator::initializeGoalPose(Action::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Reset state for new action feedback
  start_time_ = now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);
}

void
BtNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator
