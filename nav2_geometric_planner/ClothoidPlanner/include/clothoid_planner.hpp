// Copyright (c) 2025 Nav2 Contributors
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

#ifndef CLOTHOID_PLANNER__CLOTHOID_PLANNER_HPP_
#define CLOTHOID_PLANNER__CLOTHOID_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_geometric_planners
{

/**
 * @class ClothoidPlanner
 * @brief Global planner that builds paths from G1-continuous clothoid segments.
 *
 * The planner threads clothoid curves through the ordered sequence
 * [start] + viapoints + [goal].  Curvature at each waypoint is estimated from
 * the neighbouring chord directions.  Any path that passes through a lethal
 * obstacle raises nav2_core::NoValidPathCouldBeFound.
 */
class ClothoidPlanner : public nav2_core::GlobalPlanner
{
public:
  ClothoidPlanner() = default;
  ~ClothoidPlanner() override = default;

  /// @brief Initialise the planner with ROS node, name, and costmap.
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief Create a clothoid path from start to goal through viapoints.
   * @param start  Starting pose (used as first waypoint).
   * @param goal   Goal pose (used as last waypoint).
   * @param viapoints Intermediate poses; may be empty.
   * @param cancel_checker Callable that returns true when the action is cancelled.
   * @return Planned path.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    std::function<bool()> cancel_checker) override;

private:
  /// @brief Runtime parameters for the clothoid planner.
  struct Params
  {
    double step_size_{0.05};                ///< Arc-length sampling interval (m).
    double max_curvature_{1.0};             ///< Maximum allowed curvature (rad/m).
    double collision_check_resolution_{0.05}; ///< Collision check step (m).
  };

  /// @brief Load and declare ROS parameters.
  void loadParameters();

  /// @brief Wire up dynamic parameter callbacks.
  void setupDynamicParameters();

  /**
   * @brief Validate start/goal occupancy; throws on failure.
   * @param start Start pose.
   * @param goal  Goal pose.
   */
  void validateInputs(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Build clothoid segments through all ordered waypoints.
   * @param waypoints Ordered vector beginning with start, ending with goal.
   * @return Fully assembled path.
   */
  nav_msgs::msg::Path planThroughWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints);

  /**
   * @brief Sample one clothoid segment and append valid poses to the path.
   * @param from  Start of segment.
   * @param to    End of segment.
   * @param path  Output path to append to.
   */
  void appendClothoidSegment(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    nav_msgs::msg::Path & path);

  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  Params params_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};

}  // namespace nav2_geometric_planners

#endif  // CLOTHOID_PLANNER__CLOTHOID_PLANNER_HPP_
