// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief Simple plugin based on WaypointTaskExecutor, lets robot to wait for a
 *        user input at waypoint arrival.
 */
class InputAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
/**
 * @brief Construct a new Input At Waypoint Arrival object
 *
 */
  InputAtWaypoint();

  /**
   * @brief Destroy the Input At Waypoint Arrival object
   *
   */
  ~InputAtWaypoint();

  /**
   * @brief declares and loads parameters used
   * @param parent parent node
   * @param plugin_name name of plugin
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  /**
   * @brief Processor
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return if task execution failed
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  /**
   * @brief Processor callback
   * @param msg Empty message
   */
  void Cb(const std_msgs::msg::Empty::SharedPtr msg);

  bool input_received_;
  bool is_enabled_;
  rclcpp::Duration timeout_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
