/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Fetullah Atas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.hpp>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav2_waypoint_follower/plugins/wait_at_waypoint_arrival.hpp"

namespace nav2_waypoint_follower
{
WaitAtWaypointArrival::WaitAtWaypointArrival()
  : waypoint_pause_duration_(0),
    is_enabled_(false)
{
}

WaitAtWaypointArrival::~WaitAtWaypointArrival()
{
}

void WaitAtWaypointArrival::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".waypoint_pause_duration",
    rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(false));
  node->get_parameter(
    plugin_name + ".waypoint_pause_duration",
    waypoint_pause_duration_);
  node->get_parameter(
    plugin_name + ".enabled",
    is_enabled_);
  if (!is_enabled_) {
    RCLCPP_INFO(
      node->get_logger(), "Waypoint task executor plugin is disabled.");
  }
}

void WaitAtWaypointArrival::processAtWaypoint(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const geometry_msgs::msg::PoseStamped curr_pose, const int curr_waypoint_index)
{
  if (!is_enabled_) {
    return;
  }

  auto node = parent.lock();

  RCLCPP_INFO(
    node->get_logger(), "Arrived at %i'th waypoint, sleeping for %i milliseconds",
    curr_waypoint_index,
    waypoint_pause_duration_);
  RCLCPP_INFO(
    node->get_logger(), "current Pose: x %.2f , y %.2f", curr_pose.pose.position.x,
    curr_pose.pose.position.y);
  rclcpp::sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::WaitAtWaypointArrival,
  nav2_core::TaskExecutorAtWaypointArrival)
