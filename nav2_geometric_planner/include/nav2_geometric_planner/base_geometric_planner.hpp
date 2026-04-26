// Copyright (c) 2026 Sanchit Badamikar
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

#ifndef NAV2_GEOMETRIC_PLANNER__BASE_GEOMETRIC_PLANNER_HPP_
#define NAV2_GEOMETRIC_PLANNER__BASE_GEOMETRIC_PLANNER_HPP_

#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_geometric_planners
{

/**
 * @class BaseGeometricPlanner
 * @brief Abstract interface for pure-geometry path solvers.
 *
 * Implementors compute a path from geometric inputs only — no costmap,
 * no collision checking.  The owning GeometricPlanner plugin handles all
 * ROS and costmap concerns.
 */
class BaseGeometricPlanner
{
public:
  virtual ~BaseGeometricPlanner() = default;

  /**
   * @brief Declare and read solver-specific ROS parameters.
   *
   * Called once during GeometricPlanner::configure().  The default
   * implementation is a no-op for solvers that need no parameters.
   *
   * @param parent Weak pointer to the owning lifecycle node.
   * @param name   Plugin name used as the ROS parameter namespace.
   */
  virtual void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name)
  {
    (void)parent;
    (void)name;
  }

  /**
   * @brief Compute a geometric path from start to goal.
   *
   * No collision checking is performed here.  Implementations may throw
   * nav2_core planner exceptions (e.g. InsufficientViapoints, InvalidViapoints)
   * when inputs are geometrically invalid.
   *
   * @param start      Starting pose.
   * @param goal       Goal pose.
   * @param viapoints  Intermediate waypoints/control points.
   * @param resolution Costmap cell size (metres); solvers that sample by
   *                   distance use this as the maximum pose spacing.
   * @return Sampled path from start to goal.
   */
  virtual nav_msgs::msg::Path createPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    double resolution) = 0;
};

}  // namespace nav2_geometric_planners

#endif  // NAV2_GEOMETRIC_PLANNER__BASE_GEOMETRIC_PLANNER_HPP_
