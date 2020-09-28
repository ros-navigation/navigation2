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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_ARRIVAL_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_ARRIVAL_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/task_executor_at_waypoint_arrival.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief Simple plugin based on TaskExecutorAtWaypointArrival, lets robot to sleep for a
 *        specified amount of time at waypoint arrival. You can reference this class to define
 *        your own task and rewrite the body for it.
 *
 */
class WaitAtWaypointArrival : public nav2_core::TaskExecutorAtWaypointArrival
{
private:
  // the robot will sleep waypoint_pause_duration_ milliseconds
  int waypoint_pause_duration_;
  bool is_enabled_;

public:
/**
 * @brief Construct a new Wait At Waypoint Arrival object
 *
 */
  WaitAtWaypointArrival();

  /**
   * @brief Destroy the Wait At Waypoint Arrival object
   *
   */
  ~WaitAtWaypointArrival();

  /**
   * @brief declares and loads parameters used (waypoint_pause_duration_)
   *
   * @param parent parent node that plugin will be created withing(waypoint_follower in this case)
   * @param plugin_name
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param parent parent node that plugin will be created withing(waypoint_follower in this case)
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   */
  void processAtWaypoint(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const geometry_msgs::msg::PoseStamped curr_pose, const int curr_waypoint_index);
};

}  // namespace nav2_waypoint_follower
#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_ARRIVAL_HPP_
