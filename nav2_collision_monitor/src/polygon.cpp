// Copyright (c) 2022 Samsung Research Russia
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

#include "nav2_collision_monitor/polygon.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  nav2_util::LifecycleNode * node, const EmergencyModel em, const std::vector<Point> poly)
: poly_(poly)
{
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), "Creating Polygon");

  polygon_type_ = POLYGON;
  emergency_model_ = em;
}

Polygon::~Polygon()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying Polygon");
  poly_.clear();
}

void Polygon::getPoly(std::vector<Point> & poly)
{
  poly = poly_;
}

bool Polygon::isPointInside(const Point & point)
{
  // Implementation of crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result

  // In ROS polygon has the last vertex is equal to first, so it is not considered
  j = poly_size - 2;
  for (i = 0; i < poly_size - 1; i++) {
    // Testing the edge only if given point is between edge boundaries by Y coordinates
    if ((point.y < poly_[i].y) != (point.y < poly_[j].y)) {
      // Calculating intersection coordinate of X+ ray
      const double x_inter = poly_[i].x +
        (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) /
        (poly_[j].y - poly_[i].y);
      // If intersection with tested edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    j = i;
  }
  return res;
}

}  // namespace nav2_collision_monitor
