// Copyright (c) 2025
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__WAYPOINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__WAYPOINT_CRITIC_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::WaypointCritic
 * @brief Critic objective function for following sparse waypoints
 * 
 * This critic subscribes to a nav_msgs/Path topic containing sparse waypoints
 * and penalizes trajectories that don't pass through these waypoints with
 * configurable position and orientation tolerances.
 */
class WaypointCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to waypoint following
   *
   * @param costs [out] add waypoint cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  /**
   * @brief Callback for waypoint path subscription
   * @param msg Received path message containing waypoints
   */
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Get the current target waypoint (always the first one in the list)
   * @return Index of target waypoint (0 for first), or -1 if no waypoints available
   */
  int getTargetWaypoint();

  /**
   * @brief Calculate distance between two points
   * @param x1 X coordinate of first point
   * @param y1 Y coordinate of first point
   * @param x2 X coordinate of second point
   * @param y2 Y coordinate of second point
   * @return Euclidean distance
   */
  double calculateDistance(double x1, double y1, double x2, double y2);

  /**
   * @brief Calculate angular difference between two angles
   * @param angle1 First angle in radians
   * @param angle2 Second angle in radians
   * @return Angular difference in radians (-pi to pi)
   */
  double calculateAngularDifference(double angle1, double angle2);

  // ROS subscription
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  
  // Waypoint data
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::mutex waypoints_mutex_;
  
  // Parameters
  unsigned int power_{1};
  float weight_{1.0f};
  float position_tolerance_{0.5f};  // meters
  float yaw_tolerance_{0.5f};       // radians
  std::string waypoint_topic_{"truncated_path"};
  bool use_orientation_{true};      // whether to consider orientation
  float max_waypoint_distance_{10.0f}; // max distance to consider a waypoint
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string waypoints_frame_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__WAYPOINT_CRITIC_HPP_
