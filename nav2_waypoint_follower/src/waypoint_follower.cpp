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
: nav2_util::LifecycleNode("WaypointFollower", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
}

WaypointFollower::~WaypointFollower()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
WaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();

  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
  new_args.push_back("--");
  client_node_ = std::make_shared<rclcpp::Node>(
    "_", "", rclcpp::NodeOptions().arguments(new_args));

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
    client_node_, "navigate_to_pose");

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "FollowWaypoints", std::bind(&WaypointFollower::followWaypoints, this), false);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

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
      rclcpp::spin_until_future_complete(client_node_, cancel_future);
      // for result callback processing
      spin_some(client_node_);
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
        get_logger(), "Succeeded processing waypoint %i, "
        "moving to next.", goal_index);
    }

    if (current_goal_status_ != ActionStatus::PROCESSING &&
      current_goal_status_ != ActionStatus::UNKNOWN)
    {
      // Update server state
      goal_index++;
      new_goal = true;
      if (goal_index >= goal->poses.size()) {
        RCLCPP_INFO(
          get_logger(), "Completed all %i waypoints requested.",
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

    rclcpp::spin_some(client_node_);
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
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

}  // namespace nav2_waypoint_follower
