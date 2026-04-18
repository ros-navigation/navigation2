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

#ifndef BEZIER_PLANNER__BEZIER_PLANNER_HPP_
#define BEZIER_PLANNER__BEZIER_PLANNER_HPP_

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
#include "geometry_msgs/msg/point.hpp"

namespace nav2_geometric_planners
{

/**
 * @class BezierPlanner
 * @brief Global planner that follows a Bezier curve.
 *
 * The control-point sequence is [start] + viapoints + [goal].  At least one
 * viapoint is required (degree ≥ 2) for a meaningfully curved path.
 * The path is sampled densely and thinned so that consecutive poses are
 * separated by at most the costmap resolution.
 */
class BezierPlanner : public nav2_core::GlobalPlanner
{
public:
  BezierPlanner() = default;
  ~BezierPlanner() override = default;

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
   * @brief Create a Bezier-curve path from start to goal.
   *
   * Throws nav2_core::InsufficientViapoints if no viapoints are provided.
   *
   * @param start  Start pose (first control point).
   * @param goal   Goal pose (last control point).
   * @param viapoints Intermediate control points; at least one is required.
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
    int num_samples_{1000};  ///< Dense sampling count before resolution thinning.
  };

  /// @brief Load and declare ROS parameters.
  void loadParameters();

  /**
   * @brief Validate inputs and throw on error.
   * @param start     Start pose.
   * @param goal      Goal pose.
   * @param viapoints Intermediate control points.
   */
  void validateInputs(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints);

  /**
   * @brief Assemble control points from start, viapoints, and goal.
   * @param start     Start pose.
   * @param viapoints Intermediate poses.
   * @param goal      Goal pose.
   * @return Ordered vector of control points.
   */
  std::vector<geometry_msgs::msg::Point> buildControlPoints(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Sample the Bezier curve and thin to costmap resolution.
   * @param control_points  All control points in order.
   * @param resolution      Maximum spacing between consecutive poses.
   * @param header          Header to stamp output poses.
   * @return Sampled path.
   */
  nav_msgs::msg::Path sampleBezier(
    const std::vector<geometry_msgs::msg::Point> & control_points,
    double resolution,
    const std_msgs::msg::Header & header);

  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  Params params_;
};

}  // namespace nav2_geometric_planners

#endif  // BEZIER_PLANNER__BEZIER_PLANNER_HPP_
