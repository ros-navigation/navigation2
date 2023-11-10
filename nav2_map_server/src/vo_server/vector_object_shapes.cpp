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
#include <math.h>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/occ_grid_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/raytrace_line_2d.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_map_server
{

// ---------- Shape ----------

Shape::Shape(const nav2_util::LifecycleNode::WeakPtr & node)
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
    std::string uuid_str = getROSParameter(
      node, shape_name + ".uuid", rclcpp::PARAMETER_STRING).as_string();
    if (uuid_parse(uuid_str.c_str(), out_uuid)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[%s] Can not parse UUID string for shape: %s",
        shape_name.c_str(), uuid_str.c_str());
      return false;
    }
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
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
  const nav2_util::LifecycleNode::WeakPtr & node)
: Shape::Shape(node)
{
  type_ = POLYGON;
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

  params_->header.frame_id = getROSParameter(
    node, shape_name + ".frame_id", "map").as_string();
  params_->value = getROSParameter(
    node, shape_name + ".value", nav2_util::OCC_GRID_OCCUPIED).as_int();

  params_->closed = getROSParameter(
    node, shape_name + ".closed", true).as_bool();

  std::vector<double> poly_row;
  try {
    poly_row = getROSParameter(
      node, shape_name + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY).as_double_array();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] Error while getting polygon parameters: %s",
      shape_name.c_str(), ex.what());
    return false;
  }
  // Check for points format correctness
  if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
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

void Polygon::getBoundaries(double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  for (auto point : polygon_->points) {
    if (point.x < min_x) {
      min_x = point.x;
    }
    if (point.y < min_y) {
      min_y = point.y;
    }
    if (point.x > max_x) {
      max_x = point.x;
    }
    if (point.y > max_y) {
      max_y = point.y;
    }
  }
}

bool Polygon::isPointInside(const double px, const double py) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int points_num = polygon_->points.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = points_num - 1;
  for (j = 0; j < points_num; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((py <= polygon_->points[i].y) == (py > polygon_->points[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = polygon_->points[i].x +
        (py - polygon_->points[i].y) *
        (polygon_->points[j].x - polygon_->points[i].x) /
        (polygon_->points[j].y - polygon_->points[i].y);
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

void Polygon::putBorders(
  nav_msgs::msg::OccupancyGrid::SharedPtr map, const OverlayType overlay_type)
{
  unsigned int mx0, my0, mx1, my1;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!nav2_util::worldToMap(map, polygon_->points[0].x, polygon_->points[0].y, mx1, my1)) {
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
    if (!nav2_util::worldToMap(map, polygon_->points[i].x, polygon_->points[i].y, mx1, my1)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[UUID: %s] Can not convert (%f, %f) point to map",
        getUUID().c_str(), polygon_->points[i].x, polygon_->points[i].y);
      return;
    }
    nav2_util::raytraceLine(ma, mx0, my0, mx1, my1, map->info.width);
  }
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

bool Polygon::toFrame(
  const std::string & to_frame,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
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

// ---------- Circle ----------

Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node)
: Shape::Shape(node)
{
  type_ = CIRCLE;
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

  params_->header.frame_id = getROSParameter(
    node, shape_name + ".frame_id", "map").as_string();
  params_->value = getROSParameter(
    node, shape_name + ".value", nav2_util::OCC_GRID_OCCUPIED).as_int();

  params_->fill = getROSParameter(
    node, shape_name + ".fill", true).as_bool();

  std::vector<double> center_row;
  try {
    center_row = getROSParameter(
      node, shape_name + ".center", rclcpp::PARAMETER_DOUBLE_ARRAY).as_double_array();
    params_->radius = getROSParameter(
      node, shape_name + ".radius", rclcpp::PARAMETER_DOUBLE).as_double();
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

// Get/update shape boundaries
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
  // and to avoid any FP-accuracy loosing on small values, so we are using another
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
  fillMap(map, my * map->info.width + mx, params_->value, overlay_type);
}

// Put params_gons line borders on map
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

bool Circle::toFrame(
  const std::string & to_frame,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
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

}  // namespace nav2_map_server
