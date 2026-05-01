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

#ifndef NAV2_GEOMETRIC_PLANNER__SOLVERS__CLOTHOID_SOLVER_HPP_
#define NAV2_GEOMETRIC_PLANNER__SOLVERS__CLOTHOID_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_geometric_planner/base_geometric_planner.hpp"

namespace nav2_geometric_planners
{

/**
 * @class ClothoidSolver
 * @brief Builds paths from G1-continuous clothoid segments through all waypoints.
 *
 * Curvature at each waypoint is estimated from neighbouring chord directions.
 * Supports zero or more viapoints.
 */
class ClothoidSolver : public BaseGeometricPlanner
{
public:
  ClothoidSolver() = default;
  ~ClothoidSolver() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name) override;

  nav_msgs::msg::Path createPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    double resolution) override;

private:
  nav_msgs::msg::Path planThroughWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints);

  void appendClothoidSegment(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    nav_msgs::msg::Path & path);

  double step_size_{0.05};
  double max_curvature_{1.0};

  nav2::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};

}  // namespace nav2_geometric_planners

#endif  // NAV2_GEOMETRIC_PLANNER__SOLVERS__CLOTHOID_SOLVER_HPP_
