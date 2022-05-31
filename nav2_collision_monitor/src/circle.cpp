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

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string polygon_name,
  const double simulation_time_step)
: radius_(0.0)
{
  node_ = node;

  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Creating Circle");
  }

  polygon_type_ = CIRCLE;
  polygon_name_ = polygon_name;
  action_type_ = DO_NOTHING;

  stop_points_ = -1;
  slowdown_ = 0.0;
  time_before_collision_ = -1.0;

  simulation_time_step_ = simulation_time_step;
}

Circle::~Circle()
{
  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Destroying Circle");
  }
}

bool Circle::getParameters() {
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
      node, polygon_name_ + ".radius", rclcpp::PARAMETER_DOUBLE);
    radius_ = node->get_parameter(polygon_name_ + ".radius").as_double();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Error while getting circle parameters: %s", ex.what());
    return false;
  }

  return true;
}

void Circle::getPolygon(std::vector<Point> & poly)
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
