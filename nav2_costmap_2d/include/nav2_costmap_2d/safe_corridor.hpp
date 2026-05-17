// Copyright (c) 2026, Open Navigation LLC
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

#ifndef NAV2_COSTMAP_2D__SAFE_CORRIDOR_HPP_
#define NAV2_COSTMAP_2D__SAFE_CORRIDOR_HPP_

#include <algorithm>
#include <limits>

#include <Eigen/Core>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

namespace nav2_costmap_2d
{

/**
 * @class SafeCorridorComputer
 * @brief Utility that computes, for each point on a path, the usable corridor
 * half-width — defined as the ESDF clearance at that path point minus the
 * robot's inscribed radius, clamped to [0, max_width].
 *
 * A trajectory point is considered inside the corridor at path point k if its
 * Euclidean distance to path point k does not exceed corridor_width[k].
 *
 * This is a lightweight upper bound on the actual free-space region: the true
 * corridor may be asymmetric (narrow on one side), but using the ESDF value
 * at the path point as an isotropic bound is conservative — any trajectory
 * point that stays within this radius of the path is guaranteed to have
 * at least robot_radius clearance from every obstacle.
 *
 * Usage:
 *   auto widths = SafeCorridorComputer::computeWidths(
 *       path_x, path_y, *inflation_layer, *costmap, robot_radius, max_width);
 *   // widths[k] is the usable half-width (m) at path point k
 */
class SafeCorridorComputer
{
public:
  /**
   * @brief Compute corridor half-widths along a path.
   *
   * @param path_x  World x-coordinates of path points (Eigen array)
   * @param path_y  World y-coordinates of path points (Eigen array)
   * @param layer   InflationLayer exposing the ESDF via getDistanceToObstacle()
   * @param costmap Costmap2D for world→cell conversion
   * @param robot_radius Inscribed radius of the robot (m)
   * @param max_width    Maximum corridor half-width (m); avoids unbounded widths
   *                     in wide-open spaces
   * @return Eigen::ArrayXf of length path_x.size(), corridor widths in meters
   */
  static Eigen::ArrayXf computeWidths(
    const Eigen::ArrayXf & path_x,
    const Eigen::ArrayXf & path_y,
    const InflationLayer & layer,
    const Costmap2D & costmap,
    float robot_radius,
    float max_width = 3.0f)
  {
    const Eigen::Index n = path_x.size();
    Eigen::ArrayXf widths(n);

    const float origin_x = static_cast<float>(costmap.getOriginX());
    const float origin_y = static_cast<float>(costmap.getOriginY());
    const float resolution = static_cast<float>(costmap.getResolution());
    const unsigned int size_x = costmap.getSizeInCellsX();
    const unsigned int size_y = costmap.getSizeInCellsY();

    for (Eigen::Index k = 0; k < n; k++) {
      const float wx = path_x(k);
      const float wy = path_y(k);

      if (wx < origin_x || wy < origin_y) {
        widths(k) = 0.0f;
        continue;
      }

      const unsigned int mx = static_cast<unsigned int>((wx - origin_x) / resolution);
      const unsigned int my = static_cast<unsigned int>((wy - origin_y) / resolution);

      if (mx >= size_x || my >= size_y) {
        widths(k) = 0.0f;
        continue;
      }

      const float esdf_dist = layer.getDistanceToObstacle(mx, my);
      widths(k) = std::min(std::max(0.0f, esdf_dist - robot_radius), max_width);
    }

    return widths;
  }
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__SAFE_CORRIDOR_HPP_
