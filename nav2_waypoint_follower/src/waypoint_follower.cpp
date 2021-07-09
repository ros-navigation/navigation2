// Copyright (c) 2019 Samsung Research America
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

#include "nav2_waypoint_follower/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace nav2_waypoint_follower
{

WaypointFollower::WaypointFollower()
: nav2_util::LifecycleNode("WaypointFollower", "", false),
  waypoint_task_executor_loader_("nav2_waypoint_follower",
    "nav2_core::WaypointTaskExecutor")
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  nav2_util::declare_parameter_if_not_declared(
    this, std::string("waypoint_task_executor_plugin"),
    rclcpp::ParameterValue(std::string("wait_at_waypoint")));
  nav2_util::declare_parameter_if_not_declared(
    this, std::string("waypoint_task_executor_plugin.plugin"),
    rclcpp::ParameterValue(std::string("nav2_waypoint_follower::WaitAtWaypoint")));
}

WaypointFollower::~WaypointFollower()
{
}

nav2_util::CallbackReturn
WaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  auto node = shared_from_this();

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
  waypoint_task_executor_id_ = get_parameter("waypoint_task_executor_plugin").as_string();

  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose", callback_group_);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_waypoints", std::bind(&WaypointFollower::followWaypoints, this));

  try {
    waypoint_task_executor_type_ = nav2_util::get_plugin_type_param(
      this,
      waypoint_task_executor_id_);
    waypoint_task_executor_ = waypoint_task_executor_loader_.createUniqueInstance(
      waypoint_task_executor_type_);
    RCLCPP_INFO(
      get_logger(), "Created waypoint_task_executor : %s of type %s",
      waypoint_task_executor_id_.c_str(), waypoint_task_executor_type_.c_str());
    waypoint_task_executor_->initialize(node, waypoint_task_executor_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create waypoint_task_executor. Exception: %s", ex.what());
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  nav_to_pose_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
WaypointFollower::followWaypoints()
{
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received follow waypoint request with %i waypoints.",
    static_cast<int>(goal->poses.size()));

  if (goal->poses.size() == 0) {
    action_server_->succeeded_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);
  uint32_t goal_index = 0;
  bool new_goal = true;

  while (rclcpp::ok()) {
    // Check if asked to stop processing action
    if (action_server_->is_cancel_requested()) {
      auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
      callback_group_executor_.spin_until_future_complete(cancel_future);
      // for result callback processing
      callback_group_executor_.spin_some();
      action_server_->terminate_all();
      return;
    }

    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      goal_index = 0;
      new_goal = true;
    }

    // Check if we need to send a new goal
    if (new_goal) {
      new_goal = false;
      ClientT::Goal client_goal;
      client_goal.pose = goal->poses[goal_index];

      auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
        std::bind(&WaypointFollower::resultCallback, this, std::placeholders::_1);
      send_goal_options.goal_response_callback =
        std::bind(&WaypointFollower::goalResponseCallback, this, std::placeholders::_1);
      future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
    }

    feedback->current_waypoint = goal_index;
    action_server_->publish_feedback(feedback);

    if (current_goal_status_ == ActionStatus::FAILED) {
      failed_ids_.push_back(goal_index);

      if (stop_on_failure_) {
        RCLCPP_WARN(
          get_logger(), "Failed to process waypoint %i in waypoint "
          "list and stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server_->terminate_current(result);
        failed_ids_.clear();
        return;
      } else {
        RCLCPP_INFO(
          get_logger(), "Failed to process waypoint %i,"
          " moving to next.", goal_index);
      }
    } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
      RCLCPP_INFO(
        get_logger(), "Succeeded processing waypoint %i, processing waypoint task execution",
        goal_index);
      bool is_task_executed = waypoint_task_executor_->processAtWaypoint(
        goal->poses[goal_index], goal_index);
      RCLCPP_INFO(
        get_logger(), "Task execution at waypoint %i %s", goal_index,
        is_task_executed ? "succeeded" : "failed!");
      // if task execution was failed and stop_on_failure_ is on , terminate action
      if (!is_task_executed && stop_on_failure_) {
        failed_ids_.push_back(goal_index);
        RCLCPP_WARN(
          get_logger(), "Failed to execute task at waypoint %i "
          " stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server_->terminate_current(result);
        failed_ids_.clear();
        return;
      } else {
        RCLCPP_INFO(
          get_logger(), "Handled task execution on waypoint %i,"
          " moving to next.", goal_index);
      }
    }

    if (current_goal_status_ != ActionStatus::PROCESSING &&
      current_goal_status_ != ActionStatus::UNKNOWN)
    {
      // Update server state
      goal_index++;
      new_goal = true;
      if (goal_index >= goal->poses.size()) {
        RCLCPP_INFO(
          get_logger(), "Completed all %lu waypoints requested.",
          goal->poses.size());
        result->missed_waypoints = failed_ids_;
        action_server_->succeeded_current(result);
        failed_ids_.clear();
        return;
      }
    } else {
      RCLCPP_INFO_EXPRESSION(
        get_logger(),
        (static_cast<int>(now().seconds()) % 30 == 0),
        "Processing waypoint %i...", goal_index);
    }

    callback_group_executor_.spin_some();
    r.sleep();
  }
}

void
WaypointFollower::resultCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void
WaypointFollower::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal)
{
  if (!goal) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

}  // namespace nav2_waypoint_follower
