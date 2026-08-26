// Copyright (c) 2023 Samsung R&D Institute Russia
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

#include "nav2_map_server/vector_object_shapes.hpp"
#include "nav2_map_server/vector_object_utils.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <vector>


#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/occ_grid_utils.hpp"
#include "nav2_util/raytrace_line_2d.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"


/**
 * @brief Helper to convert world coordinates to map coordinates with boundary clamping.
 *
 * Upstream nav2_util::worldToMap enforces a strict out-of-bounds check where
 * boundary vertices exactly matching the map's max edge evaluate to mx >= size_x
 * and return false. This helper wraps the standard logic and clamps floating-point
 * boundary vertices safely to size - 1 to prevent putBorders() from aborting on
 * valid edge coordinates.
 * See: https://github.com/ros-navigation/navigation2/issues/6278
 */
namespace
{
inline bool safeWorldToMap(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
  const double wx, const double wy, unsigned int & mx, unsigned int & my)
{
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double resolution = map->info.resolution;
  const unsigned int size_x = map->info.width;
  const unsigned int size_y = map->info.height;
  // Guard against NaN/Inf coordinates: all comparisons with NaN return false,
  // so a NaN wx/wy would silently pass the bounds check below and reach the
  // cast to unsigned int — undefined behavior.  Reject non-finite inputs first.
  if (!std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  // Task 7: guard against zero-width/height map (size_x - 1 would underflow to UINT_MAX)
  if (size_x == 0 || size_y == 0) {
    return false;
  }
  const double eps = resolution * 1e-6;
  const double max_x = origin_x + size_x * resolution;
  const double max_y = origin_y + size_y * resolution;
  if (wx < origin_x || wy < origin_y || wx > max_x + eps || wy > max_y + eps) {
    return false;
  }
  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);
  if (mx >= size_x) {mx = size_x - 1;}
  if (my >= size_y) {my = size_y - 1;}
  return true;
}

inline bool clipLineSegment(
  const double min_x, const double max_x,
  const double min_y, const double max_y,
  double & x0, double & y0,
  double & x1, double & y1)
{
  double dx = x1 - x0;
  double dy = y1 - y0;

  double t0 = 0.0;
  double t1 = 1.0;

  auto clipTest = [&](double p, double q) -> bool {
      if (p == 0.0) {
        if (q < 0.0) {return false;}
      } else {
        double t = q / p;
        if (p < 0.0) {
          if (t > t1) {return false;}
          if (t > t0) {t0 = t;}
        } else {
          if (t < t0) {return false;}
          if (t < t1) {t1 = t;}
        }
      }
      return true;
    };

  if (clipTest(-dx, x0 - min_x) &&
    clipTest(dx, max_x - x0) &&
    clipTest(-dy, y0 - min_y) &&
    clipTest(dy, max_y - y0))
  {
    if (t1 < 1.0) {
      x1 = x0 + t1 * dx;
      y1 = y0 + t1 * dy;
    }
    if (t0 > 0.0) {
      x0 = x0 + t0 * dx;
      y0 = y0 + t0 * dy;
    }
    return true;
  }
  return false;
}
}  // namespace

namespace nav2_map_server
{

// ---------- Shape ----------

Shape::Shape(const nav2::LifecycleNode::WeakPtr & node)
: type_(UNKNOWN), node_(node)
{}

Shape::~Shape()
{}

ShapeType Shape::getType()
{
  return type_;
}

bool Shape::obtainShapeUUID(const std::string & shape_name, unsigned char * out_uuid)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Try to get shape UUID from ROS-parameters
    std::string uuid_str = nav2::declare_or_get_parameter<std::string>(
      node, shape_name + ".uuid");
    if (uuid_parse(uuid_str.c_str(), out_uuid) != 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[%s] Can not parse UUID string for shape: %s",
        shape_name.c_str(), uuid_str.c_str());
      return false;
    }
  } catch (const std::exception &) {
    // If no UUID was specified, generate a new one
    uuid_generate(out_uuid);

    char uuid_str[37];
    uuid_unparse(out_uuid, uuid_str);
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] No UUID is specified for shape. Generating a new one: %s",
      shape_name.c_str(), uuid_str);
  }

  return true;
}

// ---------- Polygon ----------

Polygon::Polygon(
  const nav2::LifecycleNode::WeakPtr & node)
: Shape::Shape(node)
{
  type_ = POLYGON;
}

int8_t Polygon::getValue() const
{
  return params_->value;
}

std::string Polygon::getFrameID() const
{
  return params_->header.frame_id;
}

std::string Polygon::getUUID() const
{
  return unparseUUID(params_->uuid.uuid.data());
}

bool Polygon::isUUID(const unsigned char * uuid) const
{
  return uuid_compare(params_->uuid.uuid.data(), uuid) == 0;
}

bool Polygon::isFill() const
{
  return params_->closed;
}

bool Polygon::obtainParams(const std::string & shape_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!params_) {
    params_ = std::make_shared<nav2_msgs::msg::PolygonObject>();
  }
  if (!polygon_) {
    polygon_ = std::make_shared<geometry_msgs::msg::Polygon>();
  }

  params_->header.frame_id = nav2::declare_or_get_parameter(
    node, shape_name + ".frame_id", std::string{"map"});
  params_->value = nav2::declare_or_get_parameter(
    node, shape_name + ".value", static_cast<int>(nav2_util::OCC_GRID_OCCUPIED));
  params_->closed = nav2::declare_or_get_parameter(
    node, shape_name + ".closed", true);

  std::vector<double> poly_row;
  try {
    poly_row = nav2::declare_or_get_parameter<std::vector<double>>(
      node, shape_name + ".points");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Error while getting polygon parameters: %s",
      shape_name.c_str(), ex.what());
    return false;
  }
  // Check for points format correctness
  if (poly_row.size() < 6 || poly_row.size() % 2 != 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Polygon has incorrect points description",
      shape_name.c_str());
    return false;
  }

  // Obtain polygon vertices
  geometry_msgs::msg::Point32 point;
  bool first = true;
  for (double val : poly_row) {
    if (first) {
      point.x = val;
    } else {
      point.y = val;
      params_->points.push_back(point);
    }
    first = !first;
  }

  // Filling the polygon_ with obtained points in map's frame
  polygon_->points = params_->points;

  // Getting shape UUID
  return obtainShapeUUID(shape_name, params_->uuid.uuid.data());
}

nav2_msgs::msg::PolygonObject::SharedPtr Polygon::getParams() const
{
  return params_;
}

bool Polygon::setParams(const nav2_msgs::msg::PolygonObject::SharedPtr params)
{
  params_ = params;

  if (!polygon_) {
    polygon_ = std::make_shared<geometry_msgs::msg::Polygon>();
  }
  polygon_->points = params_->points;

  // If no UUID was specified, generate a new one
  if (uuid_is_null(params_->uuid.uuid.data())) {
    uuid_generate(params_->uuid.uuid.data());
  }

  return checkConsistency();
}

bool Polygon::toFrame(
  const std::string & to_frame,
  const nav2::TransformBuffer::SharedPtr tf_buffer,
  const double transform_tolerance)
{
  geometry_msgs::msg::PoseStamped from_pose, to_pose;
  from_pose.header = params_->header;
  for (unsigned int i = 0; i < params_->points.size(); i++) {
    from_pose.pose.position.x = params_->points[i].x;
    from_pose.pose.position.y = params_->points[i].y;
    from_pose.pose.position.z = params_->points[i].z;
    if (
      nav2_util::transformPoseInTargetFrame(
        from_pose, to_pose, *tf_buffer, to_frame, transform_tolerance))
    {
      polygon_->points[i].x = to_pose.pose.position.x;
      polygon_->points[i].y = to_pose.pose.position.y;
      polygon_->points[i].z = to_pose.pose.position.z;
    } else {
      return false;
    }
  }

  return true;
}

void Polygon::getBoundaries(double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  for (auto point : polygon_->points) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
}

bool Polygon::isPointInside(const double px, const double py) const
{
  return nav2_util::geometry_utils::isPointInsidePolygon(px, py, polygon_->points);
}

void Polygon::putBorders(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const auto & pts = polygon_->points;
  const std::size_t n = pts.size();
  if (n < 2) {
    return;
  }

  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double res = map->info.resolution;
  const double min_x = origin_x;
  const double max_x = origin_x + map->info.width * res;
  const double min_y = origin_y;
  const double max_y = origin_y + map->info.height * res;

  MapAction ma(map, params_->value, overlay_type);

  // If closed is true, loop wraps around back to vertex 0.
  const std::size_t num_segments = params_->closed ? n : n - 1;

  for (std::size_t i = 0; i < num_segments; i++) {
    std::size_t j = (i + 1) % n;

    double wx0 = pts[i].x;
    double wy0 = pts[i].y;
    double wx1 = pts[j].x;
    double wy1 = pts[j].y;

    if (!std::isfinite(wx0) || !std::isfinite(wy0) || !std::isfinite(wx1) || !std::isfinite(wy1)) {
      continue;
    }

    if (!clipLineSegment(min_x, max_x, min_y, max_y, wx0, wy0, wx1, wy1)) {
      continue;  // Segment is completely out-of-bounds
    }

    unsigned int mx0, my0, mx1, my1;
    if (safeWorldToMap(map, wx0, wy0, mx0, my0) && safeWorldToMap(map, wx1, wy1, mx1, my1)) {
      nav2_util::raytraceLine(ma, mx0, my0, mx1, my1, map->info.width);
    }
  }
}

bool Polygon::checkConsistency()
{
  if (params_->points.size() < 3) {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Polygon has incorrect number of vertices: %li",
      getUUID().c_str(), params_->points.size());
    return false;
  }

  return true;
}

void Polygon::putFilled(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const auto & pts = polygon_->points;
  const std::size_t n = pts.size();
  if (n < 3) {
    return;
  }

  // Rasterize the polygon using a classic scanline fill algorithm.
  //
  // This follows the same general scanline rasterization approach used by
  // graphics libraries such as OpenCV, but is implemented locally to avoid
  // introducing an OpenCV dependency while preserving the existing polygon
  // filling semantics.
  //
  // Convert all polygon vertices to continuous map-cell coordinates.
  // Using continuous coordinates perfectly matches isPointInside() math.
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double res = map->info.resolution;
  const int map_h = static_cast<int>(map->info.height);
  const int map_w = static_cast<int>(map->info.width);

  std::vector<double> vx(n), vy(n);
  for (std::size_t i = 0; i < n; i++) {
    vx[i] = (pts[i].x - origin_x) / res - 0.5;
    vy[i] = (pts[i].y - origin_y) / res - 0.5;
  }

  // Task 7: Guard against NaN vertices (e.g. from a bad TF result). A NaN
  // violates std::sort's strict-weak-ordering and can walk off the buffer.
  for (std::size_t i = 0; i < n; i++) {
    if (!std::isfinite(vx[i]) || !std::isfinite(vy[i])) {
      RCLCPP_WARN(
        node->get_logger(),
        "[UUID: %s] Polygon has non-finite vertex at index %zu after TF — skipping fill",
        getUUID().c_str(), i);
      return;
    }
  }

  // Task 7: Clamp in double space before casting to int to avoid UB when the
  // polygon extends far outside the map (e.g. bad TF result).
  const double map_h_d = static_cast<double>(map_h - 1);
  const double map_w_d = static_cast<double>(map_w - 1);

  double y_min_d = std::ceil(*std::min_element(vy.begin(), vy.end()));
  double y_max_d = std::floor(*std::max_element(vy.begin(), vy.end()));
  y_min_d = std::clamp(y_min_d, 0.0, map_h_d);
  y_max_d = std::clamp(y_max_d, 0.0, map_h_d);
  int y_min = static_cast<int>(y_min_d);
  int y_max = static_cast<int>(y_max_d);

  if (y_min > y_max) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      1000,
      "[UUID: %s] Polygon has no visible extent in Y (sub-cell or off-map) — skipping fill",
      getUUID().c_str());
    return;
  }

  // Optimization 2: Precompute per-edge information.
  // Store xi, yi, dx (=xj-xi), dy (=yj-yi) per edge.
  // The intersection at scanline y is computed as xi + (y - yi) * dx / dy,
  // preserving the original nav2 multiply-then-divide order for bit-identical
  // output on boundary cases (Task 3).
  struct EdgeInfo
  {
    double y_lo;  // lower (exclusive) Y bound — matches isPointInsidePolygon (y_lo, y_hi]
    double y_hi;  // upper (inclusive) Y bound
    double xi;   // X at the lower-Y endpoint
    double yi;   // Y at the lower-Y endpoint
    double dx;   // xj - xi
    double dy;   // yj - yi  (always > 0 after orientation normalisation)
  };
  std::vector<EdgeInfo> edges;
  edges.reserve(n);
  for (std::size_t i = 0; i < n; i++) {
    std::size_t j = (i + 1) % n;
    double y0 = vy[i], y1 = vy[j];
    double x0 = vx[i], x1 = vx[j];
    if (y0 == y1) {
      continue;  // horizontal edge — never contributes an intersection
    }
    EdgeInfo e;
    // Normalise so dy > 0 (low-to-high) to keep xi/yi at the lower endpoint.
    if (y0 < y1) {
      e.y_lo = y0;  e.y_hi = y1;  e.xi = x0;  e.yi = y0;
      e.dx = x1 - x0;  e.dy = y1 - y0;
    } else {
      e.y_lo = y1;  e.y_hi = y0;  e.xi = x1;  e.yi = y1;
      e.dx = x0 - x1;  e.dy = y0 - y1;
    }
    edges.push_back(e);
  }

  const int8_t fill_val = params_->value;

  // Optimization 1: Allocate the intersection vector once outside the loop.
  // Reserve the maximum possible intersections (one per edge) so that no
  // heap allocation occurs during the scanline sweep.
  std::vector<double> xs;
  xs.reserve(edges.size());

  for (int y = y_min; y <= y_max; y++) {
    // Collect intersections for this scanline using precomputed edge info.
    xs.clear();
    for (const auto & e : edges) {
      // Task 2: Use half-open interval (y_lo, y_hi] — lower exclusive, upper
      // inclusive — matching nav2_util::geometry_utils::isPointInsidePolygon().
      if (y <= e.y_lo || y > e.y_hi) {
        continue;
      }
      // Task 3: Compute intersection with multiply-then-divide order to match
      // the nav2 formula: xi + (py - yi) * (xj - xi) / (yj - yi).
      xs.push_back(e.xi + (y - e.yi) * e.dx / e.dy);
    }

    std::sort(xs.begin(), xs.end());

    for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
      const double a = std::ceil(xs[k]);
      const double b = std::ceil(xs[k + 1]) - 1.0;
      if (b < 0.0 || a > map_w_d) {
        continue;
      }
      const int x_start = static_cast<int>(std::max(a, 0.0));
      const int x_end = static_cast<int>(std::min(b, map_w_d));

      if (x_start > x_end) {
        continue;
      }

      // Optimization 3: For the common OVERLAY_SEQ case, fill the span
      // with a single std::fill instead of a per-pixel processCell loop.
      const unsigned int row_offset = static_cast<unsigned int>(y) * map->info.width;
      if (overlay_type == OverlayType::OVERLAY_SEQ) {
        std::fill(
          map->data.begin() + row_offset + x_start,
          map->data.begin() + row_offset + x_end + 1,
          fill_val);
      } else {
        for (int x = x_start; x <= x_end; x++) {
          processCell(map, row_offset + static_cast<unsigned int>(x), fill_val, overlay_type);
        }
      }
    }
  }
}

// ---------- Circle ----------

Circle::Circle(
  const nav2::LifecycleNode::WeakPtr & node)
: Shape::Shape(node)
{
  type_ = CIRCLE;
}

int8_t Circle::getValue() const
{
  return params_->value;
}

std::string Circle::getFrameID() const
{
  return params_->header.frame_id;
}

std::string Circle::getUUID() const
{
  return unparseUUID(params_->uuid.uuid.data());
}

bool Circle::isUUID(const unsigned char * uuid) const
{
  return uuid_compare(params_->uuid.uuid.data(), uuid) == 0;
}

bool Circle::isFill() const
{
  return params_->fill;
}

bool Circle::obtainParams(const std::string & shape_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!params_) {
    params_ = std::make_shared<nav2_msgs::msg::CircleObject>();
  }
  if (!center_) {
    center_ = std::make_shared<geometry_msgs::msg::Point32>();
  }

  params_->header.frame_id = nav2::declare_or_get_parameter(
    node, shape_name + ".frame_id", std::string{"map"});
  params_->value = nav2::declare_or_get_parameter(
    node, shape_name + ".value", static_cast<int>(nav2_util::OCC_GRID_OCCUPIED));
  params_->fill = nav2::declare_or_get_parameter(
    node, shape_name + ".fill", true);

  std::vector<double> center_row;
  try {
    center_row = nav2::declare_or_get_parameter<std::vector<double>>(
      node, shape_name + ".center");
    params_->radius = nav2::declare_or_get_parameter<double>(
      node, shape_name + ".radius");
    if (params_->radius < 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[%s] Circle has incorrect radius less than zero",
        shape_name.c_str());
      return false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Error while getting circle parameters: %s",
      shape_name.c_str(), ex.what());
    return false;
  }
  // Check for points format correctness
  if (center_row.size() != 2) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Circle has incorrect center description",
      shape_name.c_str());
    return false;
  }

  // Obtain circle center
  params_->center.x = center_row[0];
  params_->center.y = center_row[1];
  // Setting the center_ with obtained circle center in map's frame
  *center_ = params_->center;

  // Getting shape UUID
  return obtainShapeUUID(shape_name, params_->uuid.uuid.data());
}

nav2_msgs::msg::CircleObject::SharedPtr Circle::getParams() const
{
  return params_;
}

bool Circle::setParams(const nav2_msgs::msg::CircleObject::SharedPtr params)
{
  params_ = params;

  if (!center_) {
    center_ = std::make_shared<geometry_msgs::msg::Point32>();
  }
  *center_ = params_->center;

  // If no UUID was specified, generate a new one
  if (uuid_is_null(params_->uuid.uuid.data())) {
    uuid_generate(params_->uuid.uuid.data());
  }

  return checkConsistency();
}

bool Circle::toFrame(
  const std::string & to_frame,
  const nav2::TransformBuffer::SharedPtr tf_buffer,
  const double transform_tolerance)
{
  geometry_msgs::msg::PoseStamped from_pose, to_pose;
  from_pose.header = params_->header;
  from_pose.pose.position.x = params_->center.x;
  from_pose.pose.position.y = params_->center.y;
  from_pose.pose.position.z = params_->center.z;
  if (
    nav2_util::transformPoseInTargetFrame(
      from_pose, to_pose, *tf_buffer, to_frame, transform_tolerance))
  {
    center_->x = to_pose.pose.position.x;
    center_->y = to_pose.pose.position.y;
    center_->z = to_pose.pose.position.z;
  } else {
    return false;
  }

  return true;
}

void Circle::getBoundaries(double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = center_->x - params_->radius;
  min_y = center_->y - params_->radius;
  max_x = center_->x + params_->radius;
  max_y = center_->y + params_->radius;
}

bool Circle::isPointInside(const double px, const double py) const
{
  return ( (px - center_->x) * (px - center_->x) + (py - center_->y) * (py - center_->y) ) <=
         params_->radius * params_->radius;
}

void Circle::putBorders(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (
    !std::isfinite(center_->x) || !std::isfinite(center_->y) || !std::isfinite(params_->radius) ||
    params_->radius < 0.0)
  {
    return;
  }

  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double res = map->info.resolution;
  const int map_w = static_cast<int>(map->info.width);
  const int map_h = static_cast<int>(map->info.height);

  const double cxf = (center_->x - origin_x) / res - 0.5;
  const double cyf = (center_->y - origin_y) / res - 0.5;
  const double r = params_->radius / res;

  const int8_t fill_val = params_->value;

  auto putPointChecked = [&](double px, double py) {
      int mx = static_cast<int>(std::floor(px + 0.5));
      int my = static_cast<int>(std::floor(py + 0.5));
      if (mx >= 0 && mx < map_w && my >= 0 && my < map_h) {
        processCell(
          map,
          static_cast<unsigned int>(my) * map->info.width + static_cast<unsigned int>(mx),
          fill_val,
          overlay_type);
      }
    };

  if (r == 0.0) {
    putPointChecked(cxf, cyf);
    return;
  }

  const double octant_limit = r / std::sqrt(2.0);
  const int max_v = static_cast<int>(std::ceil(octant_limit));

  for (int v_int = 0; v_int <= max_v; v_int++) {
    double v = static_cast<double>(v_int);
    if (v > r) {
      v = r;
    }
    double u = std::sqrt(std::max(0.0, r * r - v * v));

    putPointChecked(cxf + u, cyf + v);
    putPointChecked(cxf + v, cyf + u);
    putPointChecked(cxf - u, cyf + v);
    putPointChecked(cxf + v, cyf - u);
    putPointChecked(cxf - u, cyf - v);
    putPointChecked(cxf - v, cyf - u);
    putPointChecked(cxf + u, cyf - v);
    putPointChecked(cxf - v, cyf + u);
  }
}

bool Circle::checkConsistency()
{
  if (params_->radius < 0.0) {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Circle has incorrect radius less than zero",
      getUUID().c_str());
    return false;
  }
  return true;
}

bool Circle::centerToMap(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
  unsigned int & mcx, unsigned int & mcy)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Get center of circle in map coordinates
  if (center_->x < map->info.origin.position.x || center_->y < map->info.origin.position.y) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Can not convert (%f, %f) circle center to map",
      getUUID().c_str(), center_->x, center_->y);
    return false;
  }
  // We need the circle center to be always shifted one cell less its logical center
  // and to avoid any FP-accuracy losing on small values, so we are using another
  // than safeWorldToMap() approach
  mcx = static_cast<unsigned int>(
    std::round((center_->x - map->info.origin.position.x) / map->info.resolution)) - 1;
  mcy = static_cast<unsigned int>(
    std::round((center_->y - map->info.origin.position.y) / map->info.resolution)) - 1;
  if (mcx >= map->info.width || mcy >= map->info.height) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Can not convert (%f, %f) point to map",
      getUUID().c_str(), center_->x, center_->y);
    return false;
  }

  return true;
}

inline void Circle::putPoint(
  unsigned int mx, unsigned int my,
  nav_msgs::msg::OccupancyGrid::SharedPtr map,
  const OverlayType overlay_type)
{
  processCell(map, my * map->info.width + mx, params_->value, overlay_type);
}

void Circle::putFilled(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!std::isfinite(center_->x) || !std::isfinite(center_->y) || !std::isfinite(params_->radius)) {
    RCLCPP_WARN(
      node->get_logger(),
      "[UUID: %s] Circle has non-finite coordinates or radius after TF — skipping fill",
      getUUID().c_str());
    return;
  }

  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double res = map->info.resolution;
  const int map_w = static_cast<int>(map->info.width);
  const int map_h = static_cast<int>(map->info.height);

  // Task 6: Compute the circle center in continuous map-cell coordinates.
  // This preserves sub-cell precision discarded by the previous integer-lattice
  // approach (safeWorldToMap floors to a cell index before geometry runs).
  // Formula: cell index = (world - origin) / res, then shift by 0.5 to move
  // from cell-edge to cell-centre coordinates.
  const double cxf = (center_->x - origin_x) / res - 0.5;
  const double cyf = (center_->y - origin_y) / res - 0.5;
  const double r = params_->radius / res;

  const double map_w_d = static_cast<double>(map_w);
  const double map_h_d = static_cast<double>(map_h);

  // Task 6: Check against the circle's extent, not just its center, so a
  // circle whose center is off-map but whose body overlaps still draws.
  double y0_check_d = std::clamp(std::ceil(cyf - r), -1.0, map_h_d);
  double y1_check_d = std::clamp(std::floor(cyf + r), -1.0, map_h_d);
  double x0_check_d = std::clamp(std::ceil(cxf - r), -1.0, map_w_d);
  double x1_check_d = std::clamp(std::floor(cxf + r), -1.0, map_w_d);

  const int y0_check = static_cast<int>(y0_check_d);
  const int y1_check = static_cast<int>(y1_check_d);
  const int x0_check = static_cast<int>(x0_check_d);
  const int x1_check = static_cast<int>(x1_check_d);

  if (y0_check >= map_h || y1_check < 0 || x0_check >= map_w || x1_check < 0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      1000,
      "[UUID: %s] Circle extent is fully off-map — skipping fill",
      getUUID().c_str());
    return;
  }

  const int y0 = std::max(y0_check, 0);
  const int y1 = std::min(y1_check, map_h - 1);

  const int8_t fill_val = params_->value;

  for (int y = y0; y <= y1; y++) {
    const double t = r * r - (y - cyf) * (y - cyf);
    if (t < 0.0) {
      continue;
    }
    const double dx = std::sqrt(t);
    const double a = std::ceil(cxf - dx);
    const double b = std::floor(cxf + dx);
    if (b < 0.0 || a > map_w_d - 1.0) {
      continue;
    }
    const int x_lo = static_cast<int>(std::max(a, 0.0));
    const int x_hi = static_cast<int>(std::min(b, map_w_d - 1.0));
    if (x_lo > x_hi) {
      continue;
    }
    const unsigned int row_offset = static_cast<unsigned int>(y) * map->info.width;
    if (overlay_type == OverlayType::OVERLAY_SEQ) {
      std::fill(
        map->data.begin() + row_offset + x_lo,
        map->data.begin() + row_offset + x_hi + 1,
        fill_val);
    } else {
      for (int x = x_lo; x <= x_hi; x++) {
        processCell(
          map,
          row_offset + static_cast<unsigned int>(x),
          fill_val,
          overlay_type);
      }
    }
  }
}

}  // namespace nav2_map_server
