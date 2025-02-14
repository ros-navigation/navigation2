// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_UTIL__POLYGON_UTILS_HPP_
#define NAV2_UTIL__POLYGON_UTILS_HPP_

#include <vector>

namespace nav2_util
{

/**
 * @brief Checks if point is inside the polygon
 * @param px X-coordinate of the given point to check
 * @param py Y-coordinate of the given point to check
 * @param polygon Polygon to check if the point is inside
 * @return True if given point is inside polygon, otherwise false
 */
template<class PointT>
inline bool isPointInsidePolygon(
  const double px, const double py, const std::vector<PointT> & polygon)
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int points_num = polygon.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = points_num - 1;
  for (j = 0; j < points_num; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((py <= polygon[i].y) == (py > polygon[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = polygon[i].x +
        (py - polygon[i].y) *
        (polygon[j].x - polygon[i].x) /
        (polygon[j].y - polygon[i].y);
      // If intersection with checked edge is greater than point x coordinate,
      // inverting the result
      if (x_inter > px) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__POLYGON_UTILS_HPP_
