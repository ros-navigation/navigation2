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

#include <uuid/uuid.h>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <vector>


#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/occ_grid_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/raytrace_line_2d.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace
{
/**
 * @brief Boundary-safe world-to-map coordinate conversion for polygon/circle rasterisation.
 *
 * nav2_util::worldToMap() (nav2_util/occ_grid_utils.hpp) uses a strict
 * out-of-bounds check: if the computed mx or my equals size_x / size_y it
 * returns false.  Polygon vertices that sit exactly on the upper map edge
 * (e.g. world_x == origin_x + width * resolution) legitimately map to the
 * last valid cell (size_x-1), but the strict check rejects them, causing
 * putBorders() to abort the entire shape mid-draw.
 *
 * This helper applies a 1-ULP-safe epsilon on the outer boundary and clamps
 * mx/my to size-1 instead of returning false, matching the semantics we need
 * for border and outline rasterisation.  It does NOT modify the global
 * nav2_util::worldToMap() behaviour.  The helper is in an anonymous namespace
 * and therefore has no external linkage.
 *
 * See: https://github.com/ros-navigation/navigation2/issues/6278
 */
inline bool safeWorldToMap(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
  const double wx, const double wy, unsigned int & mx, unsigned int & my)
{
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double resolution = map->info.resolution;
  const unsigned int size_x = map->info.width;
  const unsigned int size_y = map->info.height;

  // Compute the world-space upper boundary of the map.
  // We tolerate a small multiple of the resolution (not a hard-coded 1e-9)
  // so the epsilon scales with map coarseness and stays well below one cell.
  const double eps = resolution * 1e-6;
  const double max_x = origin_x + size_x * resolution;
  const double max_y = origin_y + size_y * resolution;

  if (wx < origin_x || wy < origin_y || wx > max_x + eps || wy > max_y + eps) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);

  // Clamp upper-boundary cells that floating-point arithmetic pushed one past
  // the legal index range.
  if (mx >= size_x) {mx = size_x - 1;}
  if (my >= size_y) {my = size_y - 1;}

  return true;
}
}  // anonymous namespace

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
  unsigned int mx0, my0, mx1, my1;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!safeWorldToMap(map, polygon_->points[0].x, polygon_->points[0].y, mx1, my1)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[UUID: %s] Can not convert (%f, %f) point to map",
      getUUID().c_str(), polygon_->points[0].x, polygon_->points[0].y);
    return;
  }

  MapAction ma(map, params_->value, overlay_type);
  for (unsigned int i = 1; i < polygon_->points.size(); i++) {
    mx0 = mx1;
    my0 = my1;
    if (!safeWorldToMap(map, polygon_->points[i].x, polygon_->points[i].y, mx1, my1)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[UUID: %s] Can not convert (%f, %f) point to map",
        getUUID().c_str(), polygon_->points[i].x, polygon_->points[i].y);
      return;
    }
    nav2_util::raytraceLine(ma, mx0, my0, mx1, my1, map->info.width);
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

  // Convert all polygon vertices to continuous map-cell coordinates.
  // Using continuous coordinates perfectly matches isPointInside() math.
  std::vector<double> vx(n), vy(n);
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double res = map->info.resolution;

  for (std::size_t i = 0; i < n; i++) {
    vx[i] = (pts[i].x - origin_x) / res - 0.5;
    vy[i] = (pts[i].y - origin_y) / res - 0.5;
  }

  // Find the Y extent of the polygon in map coordinates.
  int y_min = static_cast<int>(std::ceil(*std::min_element(vy.begin(), vy.end())));
  int y_max = static_cast<int>(std::floor(*std::max_element(vy.begin(), vy.end())));
  y_min = std::max(y_min, 0);
  y_max = std::min(y_max, static_cast<int>(map->info.height) - 1);

  const int map_width = static_cast<int>(map->info.width);
  for (int y = y_min; y <= y_max; y++) {
    std::vector<double> xs;
    xs.reserve(n);

    for (std::size_t i = 0; i < n; i++) {
      std::size_t j = (i + 1) % n;

      double y0 = vy[i], y1 = vy[j];
      double x0 = vx[i], x1 = vx[j];

      if (y0 == y1) {
        continue;
      }

      // Check if scanline crosses the edge (half-open interval)
      if (y < std::min(y0, y1) || y >= std::max(y0, y1)) {
        continue;
      }

      double x_intersect = x0 + (y - y0) * (x1 - x0) / (y1 - y0);
      xs.push_back(x_intersect);
    }

    std::sort(xs.begin(), xs.end());

    for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
      // To match ray-casting, x must be: xs[k] <= x < xs[k+1]
      int x_start = static_cast<int>(std::ceil(xs[k]));
      int x_end = static_cast<int>(std::ceil(xs[k + 1])) - 1;

      x_start = std::max(x_start, 0);
      x_end = std::min(x_end, map_width - 1);

      for (int x = x_start; x <= x_end; x++) {
        processCell(
          map,
          static_cast<unsigned int>(y) * map->info.width + static_cast<unsigned int>(x),
          params_->value,
          overlay_type);
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
  unsigned int mcx, mcy;
  if (!centerToMap(map, mcx, mcy)) {
    return;
  }

  // Implementation of the circle generation algorithm, based on the following work:
  // Berthold K.P. Horn "Circle generators for display devices"
  // Computer Graphics and Image Processing 5.2 (1976): 280-288.

  // Inputs initialization
  const int r = static_cast<int>(std::round(params_->radius / map->info.resolution));
  int x = r;
  int y = 1;

  // Error initialization
  int s = -r;

  // Calculation algorithm
  while (x > y) {  // Calculating only first circle octant
    // Put 8 points in each octant reflecting symmetrically
    putPoint(mcx + x, mcy + y, map, overlay_type);
    putPoint(mcx + y, mcy + x, map, overlay_type);
    putPoint(mcx - x + 1, mcy + y, map, overlay_type);
    putPoint(mcx + y, mcy - x + 1, map, overlay_type);
    putPoint(mcx - x + 1, mcy - y + 1, map, overlay_type);
    putPoint(mcx - y + 1, mcy - x + 1, map, overlay_type);
    putPoint(mcx + x, mcy - y + 1, map, overlay_type);
    putPoint(mcx - y + 1, mcy + x, map, overlay_type);

    s = s + 2 * y + 1;
    y++;
    if (s > 0) {
      s = s - 2 * x + 2;
      x--;
    }
  }

  // Corner case for x == y: do not put end points twice
  if (x == y) {
    putPoint(mcx + x, mcy + y, map, overlay_type);
    putPoint(mcx - x + 1, mcy + y, map, overlay_type);
    putPoint(mcx - x + 1, mcy - y + 1, map, overlay_type);
    putPoint(mcx + x, mcy - y + 1, map, overlay_type);
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
  // than nav2_util::worldToMap() approach
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
  unsigned int mcx, mcy;
  if (!safeWorldToMap(map, center_->x, center_->y, mcx, mcy)) {
    return;
  }

  const double res = map->info.resolution;
  const double r = params_->radius / res;
  const double r2 = r * r;
  const int r_int = static_cast<int>(std::ceil(r));
  const int map_w = static_cast<int>(map->info.width);
  const int map_h = static_cast<int>(map->info.height);
  const int cx = static_cast<int>(mcx);
  const int cy = static_cast<int>(mcy);

  auto fill_hspan = [&](int y, int x0, int x1) {
      if (y < 0 || y >= map_h) {return;}
      x0 = std::max(x0, 0);
      x1 = std::min(x1, map_w - 1);
      for (int x = x0; x <= x1; x++) {
        processCell(
          map,
          static_cast<unsigned int>(y) * map->info.width + static_cast<unsigned int>(x),
          params_->value,
          overlay_type);
      }
    };

  for (int dy = -r_int; dy <= r_int + 1; dy++) {
    double term = r2 - (dy - 0.5) * (dy - 0.5);
    if (term < 0.0) {
      continue;
    }
    double r_rem = std::sqrt(term);
    int x0 = cx + static_cast<int>(std::ceil(-r_rem + 0.5));
    int x1 = cx + static_cast<int>(std::floor(r_rem + 0.5));
    fill_hspan(cy + dy, x0, x1);
  }
}

}  // namespace nav2_map_server
