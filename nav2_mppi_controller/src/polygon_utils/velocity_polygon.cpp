// Copyright (c) 2026
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

#include "nav2_mppi_controller/polygon_utils/velocity_polygon.hpp"

#include <cmath>
#include <utility>

#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/array_parser.hpp"

namespace mppi::polygon_utils
{

namespace
{
/**
 * @brief Parses a "[[x1,y1],[x2,y2],...]" string into a Footprint.
 */
bool parsePolygonPoints(
  const std::string & points_str,
  nav2_costmap_2d::Footprint & out,
  std::string & error_msg)
{
  std::string parse_error;
  const std::vector<std::vector<float>> vvf = nav2_util::parseVVF(points_str, parse_error);
  if (!parse_error.empty()) {
    error_msg = "error parsing points '" + points_str + "': " + parse_error;
    return false;
  }
  if (vvf.size() < 3) {
    error_msg = "polygon must have at least 3 vertices";
    return false;
  }

  nav2_costmap_2d::Footprint parsed;
  parsed.reserve(vvf.size());
  for (const auto & v : vvf) {
    if (v.size() != 2) {
      error_msg = "each point must be a pair of numbers [x, y]";
      return false;
    }
    geometry_msgs::msg::Point point;
    point.x = v[0];
    point.y = v[1];
    parsed.push_back(point);
  }

  out = std::move(parsed);
  return true;
}
}  // namespace

bool VelocityPolygon::onConfigure(const std::string & name, ParametersHandler * param_handler)
{
  name_ = name;
  parameters_handler_ = param_handler;
  auto getParam = parameters_handler_->getParamGetter(name_);

  std::vector<std::string> velocity_polygon_names;
  getParam(velocity_polygon_names, "velocity_polygons", std::vector<std::string>());
  getParam(holonomic_, "holonomic", false);

  if (velocity_polygon_names.empty()) {
    RCLCPP_ERROR(logger_, "[%s]: velocity_polygons list is empty", name_.c_str());
    return false;
  }

  for (const auto & sub_name : velocity_polygon_names) {
    auto getSubParam = parameters_handler_->getParamGetter(name_ + "." + sub_name);

    std::string points_str;
    getSubParam(points_str, "points", std::string(""));

    SubPolygon sub;
    sub.name = sub_name;
    std::string error;
    if (points_str.empty() || !parsePolygonPoints(points_str, sub.poly, error)) {
      RCLCPP_ERROR(
        logger_, "[%s.%s]: failed to parse points: %s",
        name_.c_str(), sub_name.c_str(), error.c_str());
      return false;
    }

    const auto min_max = nav2_costmap_2d::calculateMinAndMaxDistances(sub.poly);
    sub.inscribed_radius = min_max.first;
    sub.circumscribed_radius = min_max.second;

    getSubParam(sub.linear_min, "linear_min", 0.0);
    getSubParam(sub.linear_max, "linear_max", 0.0);
    getSubParam(sub.theta_min, "theta_min", 0.0);
    getSubParam(sub.theta_max, "theta_max", 0.0);
    if (holonomic_) {
      getSubParam(sub.direction_start_angle, "direction_start_angle", -M_PI);
      getSubParam(sub.direction_end_angle, "direction_end_angle", M_PI);
    }

    sub_polygons_.push_back(sub);
  }

  return true;
}

void VelocityPolygon::computeBucketThresholds(
  const std::shared_ptr<nav2_costmap_2d::InflationLayerInterface> & inflation_layer,
  double resolution)
{
  const double inflation_radius = inflation_layer->getInflationRadius();
  for (auto & sub : sub_polygons_) {
    if (sub.circumscribed_radius <= inflation_radius) {
      sub.tau_circumscribed = inflation_layer->computeCost(sub.circumscribed_radius / resolution);
    } else {
      sub.tau_circumscribed = -1.0;
    }
    if (sub.inscribed_radius <= inflation_radius) {
      sub.tau_inscribed = inflation_layer->computeCost(sub.inscribed_radius / resolution);
    } else {
      sub.tau_inscribed = -1.0;
    }
  }
}

bool VelocityPolygon::isInRange(
  double vx, double vy, double wz, const SubPolygon & sub) const
{
  // 1. Always check angular range first
  bool in_range = (wz <= sub.theta_max && wz >= sub.theta_min);

  if (holonomic_) {
    const double magnitude = std::hypot(vx, vy);
    const double direction = (magnitude > 0.0) ? std::atan2(vy, vx) : 0.0;

    in_range &= (magnitude <= sub.linear_max && magnitude >= sub.linear_min);

    if (sub.direction_start_angle <= sub.direction_end_angle) {
      in_range &= (direction >= sub.direction_start_angle &&
        direction <= sub.direction_end_angle);
    } else {
      in_range &= (direction >= sub.direction_start_angle ||
        direction <= sub.direction_end_angle);
    }
  } else {
    // 3. Non-holonomic: keep x-based behavior
    in_range &= (vx <= sub.linear_max && vx >= sub.linear_min);
  }

  return in_range;
}

const SubPolygon * VelocityPolygon::findPolygon(double vx, double vy, double wz) const
{
  for (const auto & sub : sub_polygons_) {
    if (isInRange(vx, vy, wz, sub)) {
      return &sub;
    }
  }
  return nullptr;
}

}  // namespace mppi::polygon_utils
