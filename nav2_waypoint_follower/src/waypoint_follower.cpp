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
    "FollowWaypoints", std::bind(&WaypointFollower::followWaypointsCallback, this));

  from_ll_to_map_client_ = std::make_unique<
    nav2_util::ServiceClient<robot_localization::srv::FromLL>>(
    "/fromLL",
    client_node_);

  gps_action_server_ = std::make_unique<ActionServerGPS>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "FollowGPSWaypoints", std::bind(&WaypointFollower::followGPSWaypointsCallback, this));
  // used for transfroming orientation of GPS poses to map frame
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

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
  gps_action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  gps_action_server_->deactivate();

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
  gps_action_server_.reset();
  from_ll_to_map_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
std::vector<geometry_msgs::msg::PoseStamped> WaypointFollower::getLatestGoalPoses(
  const T & action_server)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  // compile time static check to decide which block of code to be built
  if constexpr (std::is_same<T, std::unique_ptr<ActionServer>>::value)
  {
    // If normal waypoint following callback was called, we build here
    poses = action_server->get_current_goal()->poses;
  } else {
    // If GPS waypoint following callback was called, we build here
    poses = convertGPSPoses2MapPoses(
      action_server->get_current_goal()->gps_poses, shared_from_this(),
      from_ll_to_map_client_);
  }
  return poses;
}

template<typename T, typename V, typename Z>
void WaypointFollower::followWaypointsLogic(
  const T & action_server,
  const V & feedback,
  const Z & result)
{
  auto goal = action_server->get_current_goal();

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses = getLatestGoalPoses<T>(action_server);

  if (!action_server || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received follow waypoint request with %i waypoints.",
    static_cast<int>(poses.size()));

  if (poses.size() == 0) {
    action_server->succeeded_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);
  uint32_t goal_index = 0;
  bool new_goal = true;

  while (rclcpp::ok()) {
    // Check if asked to stop processing action
    if (action_server->is_cancel_requested()) {
      auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
      rclcpp::spin_until_future_complete(client_node_, cancel_future);
      // for result callback processing
      spin_some(client_node_);
      action_server->terminate_all();
      return;
    }

    // Check if asked to process another action
    if (action_server->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server->accept_pending_goal();
      poses = getLatestGoalPoses<T>(action_server);
      goal_index = 0;
      new_goal = true;
    }

    // Check if we need to send a new goal
    if (new_goal) {
      new_goal = false;
      ClientT::Goal client_goal;
      client_goal.pose = poses[goal_index];

      auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
        std::bind(
        &WaypointFollower::resultCallback<rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult>,
        this,
        std::placeholders::_1);
      send_goal_options.goal_response_callback =
        std::bind(
        &WaypointFollower::goalResponseCallback
        <rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>, this,
        std::placeholders::_1);

      future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
    }

    feedback->current_waypoint = goal_index;
    action_server->publish_feedback(feedback);

    if (current_goal_status_ == ActionStatus::FAILED) {
      failed_ids_.push_back(goal_index);

      if (stop_on_failure_) {
        RCLCPP_WARN(
          get_logger(), "Failed to process waypoint %i in waypoint "
          "list and stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server->terminate_current(result);
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
        poses[goal_index], goal_index);
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
        action_server->terminate_current(result);
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
      if (goal_index >= poses.size()) {
        RCLCPP_INFO(
          get_logger(), "Completed all %i waypoints requested.",
          poses.size());
        result->missed_waypoints = failed_ids_;
        action_server->succeeded_current(result);
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

void WaypointFollower::followWaypointsCallback()
{
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  followWaypointsLogic<std::unique_ptr<ActionServer>,
    ActionT::Feedback::SharedPtr,
    ActionT::Result::SharedPtr>(
    action_server_,
    feedback, result);
}

void WaypointFollower::followGPSWaypointsCallback()
{
  auto feedback = std::make_shared<ActionTGPS::Feedback>();
  auto result = std::make_shared<ActionTGPS::Result>();

  followWaypointsLogic<std::unique_ptr<ActionServerGPS>,
    ActionTGPS::Feedback::SharedPtr,
    ActionTGPS::Result::SharedPtr>(
    gps_action_server_,
    feedback, result);
}

template<typename T>
void WaypointFollower::resultCallback(
  const T & result)
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

template<typename T>
void WaypointFollower::goalResponseCallback(
  const T & goal)
{
  if (!goal) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

std::vector<geometry_msgs::msg::PoseStamped>
WaypointFollower::convertGPSPoses2MapPoses(
  const std::vector<nav2_msgs::msg::OrientedNavSatFix> & gps_poses,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent_node,
  const std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>> & fromll_client)
{
  RCLCPP_INFO(parent_node->get_logger(), "Converting GPS waypoints to Map Frame..");
  rclcpp::Duration transform_tolerance(0, 500);

  std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
  auto stamp = parent_node->now();

  for (auto && curr_oriented_navsat_fix : gps_poses) {
    auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
    auto response = std::make_shared<robot_localization::srv::FromLL::Response>();
    request->ll_point.latitude = curr_oriented_navsat_fix.position.latitude;
    request->ll_point.longitude = curr_oriented_navsat_fix.position.longitude;
    request->ll_point.altitude = curr_oriented_navsat_fix.position.altitude;

    fromll_client->wait_for_service((std::chrono::seconds(1)));
    auto is_conversion_succeeded = fromll_client->invoke(
      request,
      response);
    if (!is_conversion_succeeded) {
      RCLCPP_ERROR(
        parent_node->get_logger(),
        "fromLL service of robot_localization could not convert %i th GPS waypoint to"
        "Map frame, going to skip this point!"
        "Make sure you have run navsat_transform_node of robot_localization");
      continue;
    } else {
      // this poses are assumed to be on global frame (map)
      geometry_msgs::msg::PoseStamped curr_pose_map_frame;
      curr_pose_map_frame.header.frame_id = "map";
      curr_pose_map_frame.header.stamp = stamp;
      curr_pose_map_frame.pose.position.x = response->map_point.x;
      curr_pose_map_frame.pose.position.y = response->map_point.y;
      curr_pose_map_frame.pose.position.z = response->map_point.z;

      geometry_msgs::msg::PoseStamped curr_pose_utm_frame;
      curr_pose_utm_frame.pose.orientation = curr_oriented_navsat_fix.orientation;
      curr_pose_utm_frame.header.frame_id = "utm";
      try {
        tf_buffer_->transform(curr_pose_utm_frame, curr_pose_map_frame, "map");
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
          parent_node->get_logger(),
          "Exception in itm -> map transform: %s",
          ex.what());
      }
      poses_in_map_frame_vector.push_back(curr_pose_map_frame);
    }
  }
  RCLCPP_INFO(
    parent_node->get_logger(),
    "Converted all %i GPS waypoint to Map frame", poses_in_map_frame_vector.size());
  return poses_in_map_frame_vector;
}

}  // namespace nav2_waypoint_follower
