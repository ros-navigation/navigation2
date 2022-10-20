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


#ifndef NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_core
{
/**
 * @brief Base class for creating a plugin in order to perform a specific task at waypoint arrivals.
 *
 */
class WaypointTaskExecutor
{
public:
  /**
   * @brief Construct a new Simple Task Execution At Waypoint Base object
   *
   */
  WaypointTaskExecutor() {}

  /**
   * @brief Destroy the Simple Task Execution At Waypoint Base object
   *
   */
  virtual ~WaypointTaskExecutor() {}

  /**
   * @brief Override this to setup your pub, sub or any ros services that you will use in the plugin.
   *
   * @param parent parent node that plugin will be created within(for an example see nav_waypoint_follower)
   * @param plugin_name plugin name comes from parameters in yaml file
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  virtual bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) = 0;
};
}  // namespace nav2_core
#endif  // NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
