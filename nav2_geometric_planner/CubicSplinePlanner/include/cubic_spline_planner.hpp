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

#ifndef CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_PLANNER_HPP_
#define CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_PLANNER_HPP_

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
 * @class CubicSplinePlanner
 * @brief Global planner that interpolates waypoints with a natural cubic spline.
 *
 * The ROS timestamps on each PoseStamped are used as the spline parameter.
 * At least one intermediate viapoint is required (three total waypoints) for
 * a non-degenerate spline; a flat straight line is not useful for this planner.
 * The resulting path is sampled so that consecutive poses are separated by at
 * most the costmap resolution.
 */
class CubicSplinePlanner : public nav2_core::GlobalPlanner
{
public:
  CubicSplinePlanner() = default;
  ~CubicSplinePlanner() override = default;

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
   * @brief Create a cubic-spline path from start to goal through viapoints.
   *
   * Throws nav2_core::InsufficientViapoints if no viapoints are provided.
   * Throws nav2_core::InvalidViapoints if any pose has a zero timestamp or
   * timestamps are not strictly increasing.
   *
   * @param start  Starting pose (must have a valid timestamp).
   * @param goal   Goal pose (must have a valid timestamp).
   * @param viapoints Intermediate waypoints; at least one is required.
   * @param cancel_checker Callable returning true when the action is cancelled.
   * @return Planned path sampled at costmap resolution.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    std::function<bool()> cancel_checker) override;

private:
  /// @brief Runtime parameters.
  struct Params
  {
    /// @brief Override for sampling resolution (0 = use costmap resolution).
    double interpolation_resolution_{0.0};
  };

  /// @brief Load and declare ROS parameters.
  void loadParameters();

  /**
   * @brief Validate inputs and throw on error.
   * @param start      Start pose.
   * @param goal       Goal pose.
   * @param viapoints  Intermediate viapoints.
   */
  void validateInputs(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints);

  /**
   * @brief Extract timestamps (seconds) from all ordered waypoints.
   *
   * Throws nav2_core::InvalidViapoints if any stamp is zero or the sequence
   * is not strictly monotonically increasing.
   *
   * @param waypoints All waypoints in order.
   * @return Vector of times in seconds.
   */
  std::vector<double> extractTimes(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints);

  /**
   * @brief Sample the spline and produce a path.
   * @param waypoints  All waypoints including start and goal.
   * @param times      Corresponding timestamps in seconds.
   * @param resolution Maximum allowed step between consecutive poses.
   * @return Sampled path.
   */
  nav_msgs::msg::Path sampleSpline(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    const std::vector<double> & times,
    double resolution);

  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  Params params_;
};

}  // namespace nav2_geometric_planners

#endif  // CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_PLANNER_HPP_
