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

#ifndef NAV2_GEOMETRIC_PLANNER__SOLVERS__BEZIER_SOLVER_HPP_
#define NAV2_GEOMETRIC_PLANNER__SOLVERS__BEZIER_SOLVER_HPP_

#include <string>
#include <vector>

#include "nav2_geometric_planner/base_geometric_planner.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace nav2_geometric_planners
{

/**
 * @class BezierSolver
 * @brief Computes a Bezier curve path through viapoints using de Casteljau's algorithm.
 *
 * Control-point sequence: [start] + viapoints + [goal].
 * At least one viapoint is required (throws InsufficientViapoints otherwise).
 * The path is densely sampled and thinned to the given resolution.
 */
class BezierSolver : public BaseGeometricPlanner
{
public:
  BezierSolver() = default;
  ~BezierSolver() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name) override;

  nav_msgs::msg::Path createPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    double resolution) override;

private:
  void validateViapoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints);

  std::vector<geometry_msgs::msg::Point> buildControlPoints(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    const geometry_msgs::msg::PoseStamped & goal);

  nav_msgs::msg::Path sampleBezier(
    const std::vector<geometry_msgs::msg::Point> & control_points,
    double resolution,
    const std_msgs::msg::Header & header);

  int num_samples_{1000};
};

}  // namespace nav2_geometric_planners

#endif  // NAV2_GEOMETRIC_PLANNER__SOLVERS__BEZIER_SOLVER_HPP_
