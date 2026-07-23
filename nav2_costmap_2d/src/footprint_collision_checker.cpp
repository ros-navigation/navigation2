// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2025 Angsa Robotics
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
//
// Modified by: Shivang Patel (shivaang14@gmail.com)

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

// Simple 2D point structure
struct Point2D
{
  int x, y;
  Point2D(int x_val, int y_val)
  : x(x_val), y(y_val) {}
};

// Simple rectangle structure
struct Rectangle
{
  int x, y, width, height;
  Rectangle(int x_val, int y_val, int w, int h)
  : x(x_val), y(y_val), width(w), height(h) {}
};

// Calculate bounding rectangle for a set of points
Rectangle calculateBoundingRect(const std::vector<Point2D> & points)
{
  if (points.empty()) {
    return Rectangle(0, 0, 0, 0);
  }

  int min_x = points[0].x;
  int max_x = points[0].x;
  int min_y = points[0].y;
  int max_y = points[0].y;

  for (const auto & pt : points) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  return Rectangle(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

// Point-in-polygon test using ray casting algorithm
bool isPointInPolygon(int x, int y, const std::vector<Point2D> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  size_t j = polygon.size() - 1;

  for (size_t i = 0; i < polygon.size(); i++) {
    if (((polygon[i].y > y) != (polygon[j].y > y)) &&
      (x <
      (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y - polygon[i].y) +
      polygon[i].x))
    {
      inside = !inside;
    }
    j = i;
  }

  return inside;
}

template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker()
: costmap_(nullptr)
{
}

template<typename CostmapT>
FootprintCollisionChecker<CostmapT>::FootprintCollisionChecker(
  CostmapT costmap)
: costmap_(costmap)
{
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCost(
  const Footprint & footprint,
  bool check_full_area)
{
  if (footprint.empty()) {
    return static_cast<double>(NO_INFORMATION);
  }

  // Pre-convert all footprint points to map coordinates once
  std::vector<Point2D> polygon_points;
  polygon_points.reserve(footprint.size());

  for (const auto & point : footprint) {
    unsigned int mx, my;
    if (!worldToMap(point.x, point.y, mx, my)) {
      return static_cast<double>(LETHAL_OBSTACLE);
    }
    polygon_points.emplace_back(static_cast<int>(mx), static_cast<int>(my));
  }

  // Check perimeter using pre-converted coordinates
  double perimeter_cost = 0.0;
  for (size_t i = 0; i < polygon_points.size(); ++i) {
    size_t next_i = (i + 1) % polygon_points.size();
    double line_cost = lineCost(
      polygon_points[i].x, polygon_points[next_i].x,
      polygon_points[i].y, polygon_points[next_i].y);

    perimeter_cost = std::max(perimeter_cost, line_cost);

    // Early termination if lethal obstacle found
    if (perimeter_cost == static_cast<double>(LETHAL_OBSTACLE)) {
      return perimeter_cost;
    }
  }

  // If perimeter check found collision or full area check not requested, return perimeter result
  if (perimeter_cost == static_cast<double>(LETHAL_OBSTACLE) || !check_full_area) {
    return perimeter_cost;
  }

  // If no collision on perimeter and full area check requested, rasterize the full area
  Rectangle bbox = calculateBoundingRect(polygon_points);

  // Clamp bounding box to costmap dimensions for safety
  unsigned int costmap_width = costmap_->getSizeInCellsX();
  unsigned int costmap_height = costmap_->getSizeInCellsY();

  int min_x = std::max(0, bbox.x);
  int min_y = std::max(0, bbox.y);
  int max_x = std::min(static_cast<int>(costmap_width - 1), bbox.x + bbox.width - 1);
  int max_y = std::min(static_cast<int>(costmap_height - 1), bbox.y + bbox.height - 1);

  // Translate polygon points to bounding box coordinates
  std::vector<Point2D> bbox_polygon;
  bbox_polygon.reserve(polygon_points.size());
  for (const auto & pt : polygon_points) {
    bbox_polygon.emplace_back(pt.x - bbox.x, pt.y - bbox.y);
  }

  double max_cost = perimeter_cost;

  // Iterate through the clamped bounding box and check costs only for cells inside the polygon
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      // Convert to bounding box coordinates for polygon test
      int bbox_x = x - bbox.x;
      int bbox_y = y - bbox.y;

      if (isPointInPolygon(bbox_x, bbox_y, bbox_polygon)) {
        double cell_cost = pointCost(x, y);

        // Early termination if lethal obstacle found
        if (cell_cost == static_cast<double>(LETHAL_OBSTACLE)) {
          return cell_cost;
        }

        max_cost = std::max(max_cost, cell_cost);
      }
    }
  }

  return max_cost;
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::lineCost(int x0, int x1, int y0, int y1) const
{
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point

    // if in collision, no need to continue
    if (point_cost == static_cast<double>(LETHAL_OBSTACLE)) {
      return point_cost;
    }

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

template<typename CostmapT>
bool FootprintCollisionChecker<CostmapT>::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::pointCost(int x, int y) const
{
  // Bounds checking to prevent segmentation faults
  if (x < 0 || y < 0 ||
    x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
    y >= static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return static_cast<double>(LETHAL_OBSTACLE);
  }

  return static_cast<double>(costmap_->getCost(x, y));
}

template<typename CostmapT>
void FootprintCollisionChecker<CostmapT>::setCostmap(CostmapT costmap)
{
  costmap_ = costmap;
}

template<typename CostmapT>
double FootprintCollisionChecker<CostmapT>::footprintCostAtPose(
  double x, double y, double theta, const Footprint & footprint, bool check_full_area)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  Footprint oriented_footprint;
  oriented_footprint.reserve(footprint.size());
  geometry_msgs::msg::Point new_pt;
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  return footprintCost(oriented_footprint, check_full_area);
}

// declare our valid template parameters
template class FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>;
template class FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;

}  // namespace nav2_costmap_2d
