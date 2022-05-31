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

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string polygon_name,
  const double simulation_time_step)
: poly_(std::vector<Point>())
{
  node_ = node;

  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Creating Polygon");
  }

  polygon_type_ = POLYGON;
  polygon_name_ = polygon_name;
  action_type_ = DO_NOTHING;

  stop_points_ = -1;
  slowdown_ = 0.0;
  time_before_collision_ = -1.0;

  simulation_time_step_ = simulation_time_step;
}

Polygon::~Polygon()
{
  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Destroying Polygon");
  }

  poly_.clear();
}

bool Polygon::getParameters() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!PolygonBase::getParameters()) {
    return false;
  }

  try {
    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row =
      node->get_parameter(polygon_name_ + ".points").as_double_array();
    // Check for format correctness
    if (poly_row.size() <= 4 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Polygon \"%s\" has incorrect points description",
        polygon_name_.c_str());
      return false;
    }

    // Obtain polygon vertices
    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Error while getting polygon parameters: %s", ex.what());
    return false;
  }

  return true;
}

void Polygon::getPolygon(std::vector<Point> & poly)
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
