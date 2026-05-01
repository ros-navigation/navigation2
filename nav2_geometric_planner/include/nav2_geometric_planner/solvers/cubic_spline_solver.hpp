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

#ifndef NAV2_GEOMETRIC_PLANNER__SOLVERS__CUBIC_SPLINE_SOLVER_HPP_
#define NAV2_GEOMETRIC_PLANNER__SOLVERS__CUBIC_SPLINE_SOLVER_HPP_

#include <string>
#include <vector>

#include "nav2_geometric_planner/base_geometric_planner.hpp"

namespace nav2_geometric_planners
{

/**
 * @class CubicSplineSolver
 * @brief Natural cubic spline interpolating all waypoints, parameterised by ROS timestamps.
 *
 * At least one viapoint is required (throws InsufficientViapoints).
 * All waypoints must have non-zero strictly-increasing timestamps (throws InvalidViapoints).
 * Path is sampled at the given resolution (or the interpolation_resolution param if set).
 */
class CubicSplineSolver : public BaseGeometricPlanner
{
public:
  CubicSplineSolver() = default;
  ~CubicSplineSolver() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name) override;

  nav_msgs::msg::Path createPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    double resolution) override;

private:
  void validateInputs(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints);

  std::vector<double> extractTimes(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints);

  nav_msgs::msg::Path sampleSpline(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    const std::vector<double> & times,
    double resolution);

  double interpolation_resolution_{0.0};
};

}  // namespace nav2_geometric_planners

#endif  // NAV2_GEOMETRIC_PLANNER__SOLVERS__CUBIC_SPLINE_SOLVER_HPP_
