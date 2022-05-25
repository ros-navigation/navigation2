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

#include "nav2_collision_monitor/circle.hpp"

#include <math.h>
#include <cmath>

namespace nav2_collision_monitor
{

Circle::Circle(nav2_util::LifecycleNode * node, const EmergencyModel em, const double radius)
: radius_(radius)
{
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), "Creating Circle");

  polygon_type_ = CIRCLE;
  emergency_model_ = em;
}

Circle::~Circle()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying Circle");
}

void Circle::getPoly(std::vector<Point> & poly)
{
  // Number of polygon points. More edges means better approximation.
  const double polygon_edges = 20;
  // Increment of angle during points position calculation
  double angle_increment = 2 * M_PI / polygon_edges;

  // Clear polygon before filling
  poly.clear();

  // Making new polygon looks like a circle
  Point p;
  for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment) {
    p.x = radius_ * std::cos(angle);
    p.y = radius_ * std::sin(angle);
    poly.push_back(p);
  }
  // Last point should match the first
  p.x = radius_;
  p.y = 0.0;
  poly.push_back(p);
}

bool Circle::isPointInside(const Point & point)
{
  return point.x*point.x + point.y*point.y < radius_*radius_;
}

}  // namespace nav2_collision_monitor
