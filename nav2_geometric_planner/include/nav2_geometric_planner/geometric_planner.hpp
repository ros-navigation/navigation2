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

#ifndef NAV2_GEOMETRIC_PLANNER__GEOMETRIC_PLANNER_HPP_
#define NAV2_GEOMETRIC_PLANNER__GEOMETRIC_PLANNER_HPP_

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

#include "nav2_geometric_planner/base_geometric_planner.hpp"

namespace nav2_geometric_planners
{

/**
 * @class GeometricPlanner
 * @brief nav2_core::GlobalPlanner plugin that delegates path geometry to a
 *        solver selected at configuration time via the "solver" ROS parameter.
 *
 * Supported solver values: "bezier", "cubic_spline", "clothoid".
 *
 * The plugin owns all ROS and costmap concerns:
 *   1. Validates start/goal occupancy.
 *   2. Calls solver_->createPath() for the geometric path.
 *   3. Checks the returned path for collisions.
 */
class GeometricPlanner : public nav2_core::GlobalPlanner
{
public:
  GeometricPlanner() = default;
  ~GeometricPlanner() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    std::function<bool()> cancel_checker) override;

private:
  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::unique_ptr<BaseGeometricPlanner> solver_;
};

}  // namespace nav2_geometric_planners

#endif  // NAV2_GEOMETRIC_PLANNER__GEOMETRIC_PLANNER_HPP_
