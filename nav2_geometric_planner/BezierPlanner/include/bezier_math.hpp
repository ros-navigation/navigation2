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

#ifndef BEZIER_PLANNER__BEZIER_MATH_HPP_
#define BEZIER_PLANNER__BEZIER_MATH_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace nav2_geometric_planners
{

/**
 * @brief Evaluate a Bezier curve at parameter t using de Casteljau's algorithm.
 *
 * de Casteljau's algorithm iteratively interpolates between adjacent control
 * points, giving a numerically stable evaluation for any polynomial degree.
 *
 * @param control_points Ordered control points (P_0 … P_n).
 *                       P_0 is the start, P_n is the end.
 * @param t              Curve parameter in [0, 1].
 * @return               Point on the Bezier curve at parameter t.
 */
inline geometry_msgs::msg::Point evaluateBezier(
  const std::vector<geometry_msgs::msg::Point> & control_points,
  double t)
{
  // Work on a mutable copy of the control-point positions.
  std::vector<geometry_msgs::msg::Point> pts = control_points;
  const int n = static_cast<int>(pts.size());

  for (int r = 1; r < n; ++r) {
    for (int i = 0; i < n - r; ++i) {
      pts[i].x = (1.0 - t) * pts[i].x + t * pts[i + 1].x;
      pts[i].y = (1.0 - t) * pts[i].y + t * pts[i + 1].y;
    }
  }

  return pts[0];
}

}  // namespace nav2_geometric_planners

#endif  // BEZIER_PLANNER__BEZIER_MATH_HPP_
