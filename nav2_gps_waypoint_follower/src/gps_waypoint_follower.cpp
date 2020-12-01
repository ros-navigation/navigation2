// Copyright (c) 2020 Fetullah Atas
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
#include "nav2_gps_waypoint_follower/gps_waypoint_follower.hpp"

namespace nav2_gps_waypoint_follower
{
GPSWaypointFollower::GPSWaypointFollower()
: nav2_util::LifecycleNode("GPSWaypointFollower", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

GPSWaypointFollower::~GPSWaypointFollower()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
GPSWaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
  new_args.push_back("--");
  client_node_ = std::make_shared<rclcpp::Node>(
    "_", "", rclcpp::NodeOptions().arguments(new_args));

  waypoint_follower_goal_ = ClientT::Goal();
  waypoint_follower_action_client_ = rclcpp_action::create_client<ClientT>(
    client_node_, "FollowWaypoints");

  callback_group_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  from_ll_to_map_client_ =
    this->create_client<robot_localization::srv::FromLL>(
    "/fromLL",
    rmw_qos_profile_services_default,
    callback_group_);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "FollowGPSWaypoints", std::bind(&GPSWaypointFollower::followGPSWaypoints, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GPSWaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GPSWaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GPSWaypointFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  waypoint_follower_action_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
GPSWaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void GPSWaypointFollower::followGPSWaypoints()
{
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received follow GPS waypoint request with %i waypoints.",
    static_cast<int>(goal->waypoints.size()));

  std::vector<geometry_msgs::msg::PoseStamped> poses = convertGPSWaypointstoPosesinMap(
    goal->waypoints, shared_from_this(), from_ll_to_map_client_);

  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      this->get_logger(), "FollowWaypoints action server is not available."
      "make an instance of waypoint_follower is up and running");
    return;
  }
  waypoint_follower_goal_.poses = poses;

  RCLCPP_INFO(
    this->get_logger(),
    "Sending a path of %zu waypoints:", waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  auto goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  goal_options.result_callback = std::bind(
    &GPSWaypointFollower::resultCallback, this,
    std::placeholders::_1);
  goal_options.goal_response_callback = std::bind(
    &GPSWaypointFollower::goalResponseCallback, this,
    std::placeholders::_1);

  auto future_goal_handle = waypoint_follower_action_client_->async_send_goal(
    waypoint_follower_goal_, goal_options);
  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Send goal call failed");
    return;
  }
  // Get the goal handle and save so that we can check
  // on completion in the timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return;
  }
}

std::vector<geometry_msgs::msg::PoseStamped>
GPSWaypointFollower::convertGPSWaypointstoPosesinMap(
  const
  std::vector<sensor_msgs::msg::NavSatFix> & gps_waypoints,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent_node,
  const rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr & fromll_client)
{
  RCLCPP_INFO(parent_node->get_logger(), "Converting GPS waypoints to Map Frame..");
  if (!parent_node) {
    throw std::runtime_error{
            "Failed to lock node while trying to convert GPS waypoints to Map frame"};
  }
  std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
  int index_of_gps_waypoints = 0;
  for (auto && curr_gps_waypoint : gps_waypoints) {
    auto fromLLRequest = std::make_shared<robot_localization::srv::FromLL::Request>();
    fromLLRequest->ll_point.latitude = curr_gps_waypoint.latitude;
    fromLLRequest->ll_point.longitude = curr_gps_waypoint.longitude;
    fromLLRequest->ll_point.altitude = curr_gps_waypoint.altitude;
    if (!fromll_client->wait_for_service((std::chrono::seconds(10)))) {
      RCLCPP_ERROR(
        parent_node->get_logger(),
        "fromLL service from robot_localization is not available"
        "cannot convert GPS waypoints to Map frame poses,"
        "Make sure you have run navsat_transform_node of robot_localization");
      return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    RCLCPP_INFO(
      parent_node->get_logger(), "Processing for %i'th Waypoint..",
      index_of_gps_waypoints);
    auto inner_client_callback =
      [&, parent_node](rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture inner_future)
      {
        auto result = inner_future.get();
      };
    auto inner_future_result = fromll_client->async_send_request(
      fromLLRequest,
      inner_client_callback);
    // this poses are assumed to be on global frame (map)
    geometry_msgs::msg::PoseStamped curr_waypoint_in_map_frame;
    curr_waypoint_in_map_frame.header.frame_id = "map";
    curr_waypoint_in_map_frame.header.stamp = rclcpp::Clock().now();
    curr_waypoint_in_map_frame.pose.position.x = inner_future_result.get()->map_point.x;
    curr_waypoint_in_map_frame.pose.position.y = inner_future_result.get()->map_point.y;
    curr_waypoint_in_map_frame.pose.position.z = inner_future_result.get()->map_point.z;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_msg, quat_tf);
    curr_waypoint_in_map_frame.pose.orientation = quat_msg;

    RCLCPP_INFO(
      parent_node->get_logger(),
      "%i th Waypoint Long, Lat: %.8f , %.8f converted to "
      "Map Point X,Y: %.8f , %.8f", index_of_gps_waypoints, fromLLRequest->ll_point.longitude,
      fromLLRequest->ll_point.latitude, curr_waypoint_in_map_frame.pose.position.x,
      curr_waypoint_in_map_frame.pose.position.y);
    index_of_gps_waypoints++;
    poses_in_map_frame_vector.push_back(curr_waypoint_in_map_frame);
  }
  RCLCPP_INFO(
    parent_node->get_logger(),
    "Converted all %i GPS waypoint to Map frame", poses_in_map_frame_vector.size());
  return poses_in_map_frame_vector;
}

void GPSWaypointFollower::resultCallback(
  const rclcpp_action::ClientGoalHandle
  <ClientT>::WrappedResult & result)
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

  RCLCPP_INFO(this->get_logger(), "Result received");
  for (auto number : result.result->missed_waypoints) {
    RCLCPP_INFO(
      this->get_logger(),
      "Missed"
      "%d points from given Yaml waypoints", number);
  }
}

void GPSWaypointFollower::goalResponseCallback(
  const
  rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal)
{
  if (!goal) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}
}  // namespace nav2_gps_waypoint_follower

/**
 * @brief Entry point for Way Point following demo Node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared
    <nav2_gps_waypoint_follower::GPSWaypointFollower>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
