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

#include "nav2_waypoint_follower/plugins/wait_at_waypoint.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{
WaitAtWaypoint::WaitAtWaypoint()
: waypoint_pause_duration_(0),
  is_enabled_(true)
{
}

WaitAtWaypoint::~WaitAtWaypoint()
{
}

void WaitAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".waypoint_pause_duration",
    rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  node->get_parameter(
    plugin_name + ".waypoint_pause_duration",
    waypoint_pause_duration_);
  node->get_parameter(
    plugin_name + ".enabled",
    is_enabled_);
  if (waypoint_pause_duration_ == 0) {
    is_enabled_ = false;
    RCLCPP_INFO(
      logger_,
      "Waypoint pause duration is set to zero, disabling task executor plugin.");
  } else if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Waypoint task executor plugin is disabled.");
  }
}

bool WaitAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint, sleeping for %i milliseconds",
    curr_waypoint_index,
    waypoint_pause_duration_);
  clock_->sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
  return true;
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::WaitAtWaypoint,
  nav2_core::WaypointTaskExecutor)
