// Copyright (c) 2026, Aniruddh Yelluri
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

#include "nav2_costmap_2d/geofence_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/raytrace_line_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::GeofenceLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using rcl_interfaces::msg::ParameterType;

namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kMaxCellsPerAxis = 1e5;
constexpr double kMaxTotalCells = 1e8;

bool isEqual(double a, double b)
{
  return std::abs(a - b) < kEpsilon;
}

template<typename PointT>
double crossProduct(const PointT & a, const PointT & b, const PointT & p)
{
  return (static_cast<double>(b.x) - a.x) * (static_cast<double>(p.y) - a.y) -
         (static_cast<double>(b.y) - a.y) * (static_cast<double>(p.x) - a.x);
}

// Returns true only for proper (straddling) crossings; touching or collinear
// segments return false.
template<typename PointT>
bool segmentsIntersect(
  const PointT & p1, const PointT & p2,
  const PointT & q1, const PointT & q2)
{
  const double d1 = crossProduct(q1, q2, p1);
  const double d2 = crossProduct(q1, q2, p2);
  const double d3 = crossProduct(p1, p2, q1);
  const double d4 = crossProduct(p1, p2, q2);
  return ((d1 > kEpsilon && d2 < -kEpsilon) || (d1 < -kEpsilon && d2 > kEpsilon)) &&
         ((d3 > kEpsilon && d4 < -kEpsilon) || (d3 < -kEpsilon && d4 > kEpsilon));
}

// Checks: at least 3 vertices, finite coordinates, non-zero area, and no
// proper crossings between non-adjacent edges.
template<typename PointT>
bool isValidPolygon(const std::vector<PointT> & polygon, std::string & reason)
{
  if (polygon.size() < 3) {
    reason = "Geofence polygon must have at least 3 vertices";
    return false;
  }

  for (const auto & v : polygon) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
      reason = "Geofence polygon contains non-finite coordinates";
      return false;
    }
  }

  double area = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & s = polygon[i];
    const auto & e = polygon[(i + 1) % polygon.size()];
    area += static_cast<double>(s.x) * e.y - static_cast<double>(e.x) * s.y;
  }
  if (std::abs(area) < kEpsilon) {
    reason = "Geofence polygon must enclose a non-zero area";
    return false;
  }

  for (size_t i = 0; i < polygon.size(); ++i) {
    const size_t ni = (i + 1) % polygon.size();
    for (size_t j = i + 1; j < polygon.size(); ++j) {
      const size_t nj = (j + 1) % polygon.size();
      if (ni == j || nj == i) {
        continue;
      }
      if (segmentsIntersect(polygon[i], polygon[ni], polygon[j], polygon[nj])) {
        reason = "Geofence polygon must not self-intersect";
        return false;
      }
    }
  }
  return true;
}

template<typename PointContainer>
void computeAabb(
  const PointContainer & points,
  double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = min_y = std::numeric_limits<double>::max();
  max_x = max_y = std::numeric_limits<double>::lowest();
  for (const auto & p : points) {
    min_x = std::min(min_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_x = std::max(max_x, static_cast<double>(p.x));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }
}

struct MapBounds
{
  double origin_x;
  double origin_y;
  unsigned int size_x;
  unsigned int size_y;
};

template<typename PointContainer>
bool computeFenceBounds(
  const PointContainer & points,
  double resolution,
  double circumscribed_radius,
  MapBounds & bounds,
  std::string & reason)
{
  double min_x, min_y, max_x, max_y;
  computeAabb(points, min_x, min_y, max_x, max_y);
  const double padding = std::ceil(
    (circumscribed_radius + resolution) / resolution) * resolution;
  bounds.origin_x = std::floor((min_x - padding) / resolution) * resolution;
  bounds.origin_y = std::floor((min_y - padding) / resolution) * resolution;
  const double end_x = std::ceil((max_x + padding) / resolution) * resolution;
  const double end_y = std::ceil((max_y + padding) / resolution) * resolution;
  const double cells_x_d = std::max(1.0, std::ceil((end_x - bounds.origin_x) / resolution));
  const double cells_y_d = std::max(1.0, std::ceil((end_y - bounds.origin_y) / resolution));
  if (cells_x_d > kMaxCellsPerAxis || cells_y_d > kMaxCellsPerAxis ||
    (cells_x_d * cells_y_d) > kMaxTotalCells)
  {
    reason = "Geofence requires too many cells, exceeding limit";
    return false;
  }
  bounds.size_x = static_cast<unsigned int>(cells_x_d);
  bounds.size_y = static_cast<unsigned int>(cells_y_d);
  return true;
}

// Liang-Barsky line-segment clip.
bool clipTest(double direction, double distance, double & enter, double & exit)
{
  if (direction < 0.0) {
    const double ratio = distance / direction;
    if (ratio > exit) {return false;}
    enter = std::max(enter, ratio);
  } else if (direction > 0.0) {
    const double ratio = distance / direction;
    if (ratio < enter) {return false;}
    exit = std::min(exit, ratio);
  } else if (distance < 0.0) {
    return false;
  }
  return true;
}

bool clipSegment(
  double & sx, double & sy, double & ex, double & ey,
  double min_x, double min_y, double max_x, double max_y)
{
  const double dx = ex - sx;
  const double dy = ey - sy;
  double enter = 0.0;
  double exit = 1.0;
  if (!clipTest(-dx, sx - min_x, enter, exit) ||
    !clipTest(dx, max_x - sx, enter, exit) ||
    !clipTest(-dy, sy - min_y, enter, exit) ||
    !clipTest(dy, max_y - sy, enter, exit))
  {
    return false;
  }
  ex = sx + exit * dx;
  ey = sy + exit * dy;
  sx += enter * dx;
  sy += enter * dy;
  return true;
}

struct MarkLethalCell
{
  unsigned char * costmap;
  void operator()(unsigned int offset) const {costmap[offset] = LETHAL_OBSTACLE;}
};

}  // namespace

namespace nav2_costmap_2d
{

void
GeofenceLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();
  if (layered_costmap_->isRolling()) {
    throw std::runtime_error{"GeofenceLayer only supports non-rolling global costmaps"};
  }

  Costmap2D * master = layered_costmap_->getCostmap();
  initial_size_x_ = master->getSizeInCellsX();
  initial_size_y_ = master->getSizeInCellsY();
  initial_origin_x_ = master->getOriginX();
  initial_origin_y_ = master->getOriginY();

  getParameters();
  if (!has_updated_data_) {
    current_ = true;
  }
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"GeofenceLayer: Failed to lock node"};
  }
  set_fence_service_ = node->create_service<nav2_msgs::srv::SetFence>(
    name_ + "/set_fence",
    std::bind(
      &GeofenceLayer::setFenceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void
GeofenceLayer::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"GeofenceLayer: Failed to lock node"};
  }
  enabled_ = node->declare_or_get_parameter(name_ + ".enabled", true);
  resize_to_fence_ = node->declare_or_get_parameter(name_ + ".resize_to_fence", true);

  double transform_tolerance = 0.0;
  node->get_parameter("transform_tolerance", transform_tolerance);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  const std::string polygon_string = node->declare_or_get_parameter(
    name_ + ".fence_polygon", std::string(""));
  if (polygon_string.empty() || polygon_string == "[]") {
    return;
  }

  std::vector<geometry_msgs::msg::Point> points;
  std::string reason;
  if (!makeFootprintFromString(polygon_string, points)) {
    throw std::runtime_error{"GeofenceLayer: Invalid fence_polygon format"};
  }
  if (!isValidPolygon(points, reason)) {
    throw std::runtime_error{"GeofenceLayer: Invalid fence_polygon: " + reason};
  }
  if (resize_to_fence_) {
    MapBounds bounds;
    Costmap2D * master = layered_costmap_->getCostmap();
    if (!computeFenceBounds(
        points, master->getResolution(),
        layered_costmap_->getCircumscribedRadius(), bounds, reason))
    {
      throw std::runtime_error{"GeofenceLayer: Invalid fence_polygon: " + reason};
    }
  }
  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.frame_id = global_frame_;
  polygon.polygon = toPolygon(points);
  bufferPolygon(polygon);
}

void
GeofenceLayer::activate()
{
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(&GeofenceLayer::updateParametersCallback, this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&GeofenceLayer::validateParameterUpdatesCallback, this, std::placeholders::_1));
}

void
GeofenceLayer::deactivate()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

void
GeofenceLayer::reset()
{
  has_updated_data_ = true;
  setCurrent(false);
}

void
GeofenceLayer::matchSize()
{
  CostmapLayer::matchSize();
  if (has_fence_) {
    has_updated_data_ = true;
    setCurrent(false);
  }
}

void
GeofenceLayer::setFenceCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetFence::Response> response)
{
  if (request->fence.polygon.points.empty()) {
    bufferPolygon(geometry_msgs::msg::PolygonStamped{});
    response->success = true;
    response->message = "Geofence cleared";
    return;
  }

  geometry_msgs::msg::PolygonStamped polygon_in_global_frame;
  const std::string source_frame = request->fence.header.frame_id.empty() ?
    global_frame_ : request->fence.header.frame_id;

  if (source_frame != global_frame_) {
    try {
      const tf2::TimePoint transform_time =
        rclcpp::Time(request->fence.header.stamp).nanoseconds() == 0 ?
        tf2::TimePointZero : tf2_ros::fromMsg(request->fence.header.stamp);
      const auto transform = tf_->lookupTransform(
        global_frame_, source_frame, transform_time, transform_tolerance_);
      tf2::doTransform(request->fence, polygon_in_global_frame, transform);
      polygon_in_global_frame.header.frame_id = global_frame_;
    } catch (const tf2::TransformException & exception) {
      response->success = false;
      response->message = std::string("Unable to transform geofence: ") + exception.what();
      RCLCPP_ERROR(logger_, "%s", response->message.c_str());
      return;
    }
  } else {
    polygon_in_global_frame = request->fence;
    polygon_in_global_frame.header.frame_id = global_frame_;
  }

  std::string reason;
  if (!isValidPolygon(polygon_in_global_frame.polygon.points, reason)) {
    response->success = false;
    response->message = reason;
    RCLCPP_WARN(logger_, "%s", reason.c_str());
    return;
  }
  if (resize_to_fence_) {
    MapBounds bounds;
    Costmap2D * master = layered_costmap_->getCostmap();
    if (!computeFenceBounds(
        polygon_in_global_frame.polygon.points,
        master->getResolution(),
        layered_costmap_->getCircumscribedRadius(),
        bounds, reason))
    {
      response->success = false;
      response->message = reason;
      RCLCPP_WARN(logger_, "%s", reason.c_str());
      return;
    }
  }
  bufferPolygon(polygon_in_global_frame);
  response->success = true;
  response->message = "Geofence updated";
}

void
GeofenceLayer::bufferPolygon(const geometry_msgs::msg::PolygonStamped & polygon)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  polygon_buffer_ = std::make_shared<geometry_msgs::msg::PolygonStamped>(polygon);
  has_updated_data_ = true;
  setCurrent(false);
}

void
GeofenceLayer::processFence()
{
  const auto polygon = polygon_buffer_;
  polygon_buffer_.reset();

  if (polygon && polygon->polygon.points.empty()) {
    has_fence_ = false;
    polygon_points_.clear();
    Costmap2D * master = layered_costmap_->getCostmap();
    if (resize_to_fence_ &&
      (master->getSizeInCellsX() != initial_size_x_ ||
      master->getSizeInCellsY() != initial_size_y_ ||
      !isEqual(master->getOriginX(), initial_origin_x_) ||
      !isEqual(master->getOriginY(), initial_origin_y_)))
    {
      layered_costmap_->resizeMap(
        initial_size_x_, initial_size_y_, master->getResolution(),
        initial_origin_x_, initial_origin_y_, false);
    } else {
      matchSize();
    }
    resetMaps();
    return;
  }

  if (polygon) {
    polygon_points_ = toPointVector(polygon->polygon);
    has_fence_ = true;
  }
  if (!has_fence_) {
    return;
  }

  double min_x, min_y, max_x, max_y;
  computeAabb(polygon_points_, min_x, min_y, max_x, max_y);
  Costmap2D * master = layered_costmap_->getCostmap();
  const double resolution = master->getResolution();

  if (resize_to_fence_) {
    auto plugins = layered_costmap_->getPlugins();
    if (plugins && !plugins->empty() && plugins->front().get() != this) {
      RCLCPP_WARN_ONCE(
        logger_,
        "GeofenceLayer: resize_to_fence is true but layer is not first in plugins list. "
        "Resizing will reset earlier layers.");
    }

    MapBounds bounds;
    std::string reason;
    if (!computeFenceBounds(
        polygon_points_, resolution,
        layered_costmap_->getCircumscribedRadius(), bounds, reason))
    {
      RCLCPP_ERROR(logger_, "GeofenceLayer: %s", reason.c_str());
      has_fence_ = false;
      polygon_points_.clear();
      matchSize();
      resetMaps();
      return;
    }
    if (master->getSizeInCellsX() != bounds.size_x ||
      master->getSizeInCellsY() != bounds.size_y ||
      !isEqual(master->getOriginX(), bounds.origin_x) ||
      !isEqual(master->getOriginY(), bounds.origin_y))
    {
      layered_costmap_->resizeMap(
        bounds.size_x, bounds.size_y, resolution,
        bounds.origin_x, bounds.origin_y, true);
    } else {
      matchSize();
    }
  } else {
    matchSize();
  }
  rasterizeFence(min_x, min_y, max_x, max_y);
}

void
GeofenceLayer::rasterizeFence(double min_x, double min_y, double max_x, double max_y)
{
  std::fill(costmap_, costmap_ + size_x_ * size_y_, LETHAL_OBSTACLE);
  const double map_max_x = origin_x_ + size_x_ * resolution_;
  const double map_max_y = origin_y_ + size_y_ * resolution_;
  if (max_x < origin_x_ || min_x > map_max_x || max_y < origin_y_ || min_y > map_max_y) {
    return;
  }

  auto forEachEdge = [this](auto && fn) {
      for (size_t i = 0; i < polygon_points_.size(); ++i) {
        fn(polygon_points_[i], polygon_points_[(i + 1) % polygon_points_.size()]);
      }
    };

  const int min_row = static_cast<int>(std::max(0.0,
      std::floor((min_y - origin_y_) / resolution_)));
  const int max_row = static_cast<int>(std::min(
    static_cast<double>(size_y_ - 1), std::floor((max_y - origin_y_) / resolution_)));
  std::vector<double> intersections;
  intersections.reserve(polygon_points_.size());
  for (int row = min_row; row <= max_row; ++row) {
    const double world_y = origin_y_ + (row + 0.5) * resolution_;
    intersections.clear();
    forEachEdge([&](const auto & s, const auto & e) {
        if ((s.y <= world_y && e.y > world_y) || (e.y <= world_y && s.y > world_y)) {
          intersections.push_back(s.x + (world_y - s.y) * (e.x - s.x) / (e.y - s.y));
        }
    });
    std::sort(intersections.begin(), intersections.end());
    for (size_t idx = 0; idx + 1 < intersections.size(); idx += 2) {
      const int first_cell = std::max(
        0, static_cast<int>(std::ceil((intersections[idx] - origin_x_) / resolution_)));
      const int last_cell = std::min(
        static_cast<int>(size_x_) - 1,
        static_cast<int>(std::floor((intersections[idx + 1] - origin_x_) / resolution_)) - 1);
      for (int col = first_cell; col <= last_cell; ++col) {
        costmap_[getIndex(col, row)] = FREE_SPACE;
      }
    }
  }

  MarkLethalCell mark_lethal{costmap_};
  const double max_col = static_cast<double>(size_x_ - 1);
  const double max_row_d = static_cast<double>(size_y_ - 1);
  forEachEdge([&](const auto & s, const auto & e) {
      double sx = (s.x - origin_x_) / resolution_;
      double sy = (s.y - origin_y_) / resolution_;
      double ex = (e.x - origin_x_) / resolution_;
      double ey = (e.y - origin_y_) / resolution_;
      if (clipSegment(sx, sy, ex, ey, 0.0, 0.0, max_col, max_row_d)) {
        nav2_util::raytraceLine(
        mark_lethal,
        static_cast<unsigned int>(std::clamp(std::round(sx), 0.0, max_col)),
        static_cast<unsigned int>(std::clamp(std::round(sy), 0.0, max_row_d)),
        static_cast<unsigned int>(std::clamp(std::round(ex), 0.0, max_col)),
        static_cast<unsigned int>(std::clamp(std::round(ey), 0.0, max_row_d)),
        size_x_);
      }
  });
}

void
GeofenceLayer::updateBounds(
  double /* robot_x */, double /* robot_y */, double /* robot_yaw */,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!has_updated_data_) {
    return;
  }
  processFence();
  has_updated_data_ = false;
  Costmap2D * master = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, master->getOriginX());
  *min_y = std::min(*min_y, master->getOriginY());
  *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
  *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
}

void
GeofenceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (enabled_ && has_fence_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  setCurrent(true);
}

rcl_interfaces::msg::SetParametersResult
GeofenceLayer::validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == name_ + ".fence_polygon") {
      result.successful = false;
      result.reason = "fence_polygon is only read during initialization; use the set_fence service";
      return result;
    }
    if (name != name_ + ".enabled" && name != name_ + ".resize_to_fence") {
      continue;
    }
    if (parameter.get_type() != ParameterType::PARAMETER_BOOL) {
      result.successful = false;
      result.reason = name + " must be a boolean";
      return result;
    }
  }
  return result;
}

void
GeofenceLayer::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name_ + ".enabled") {
      const bool enabled = parameter.as_bool();
      if (enabled_ != enabled) {
        enabled_ = enabled;
        has_updated_data_ = true;
        setCurrent(false);
      }
    } else if (parameter.get_name() == name_ + ".resize_to_fence") {
      const bool resize_to_fence = parameter.as_bool();
      if (resize_to_fence_ != resize_to_fence) {
        resize_to_fence_ = resize_to_fence;
        has_updated_data_ = true;
        setCurrent(false);
      }
    }
  }
}

}  // namespace nav2_costmap_2d
