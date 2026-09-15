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
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::GeofenceLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

void
GeofenceLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"GeofenceLayer: Failed to lock node"};
  }

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
  resize_to_fence_ = node->declare_or_get_parameter(name_ + "." + "resize_to_fence", true);
  int border_thick_param = node->declare_or_get_parameter(name_ + "." + "border_thickness", 3);
  if (border_thick_param < 0 || border_thick_param > 1000) {
    throw std::runtime_error{
            "GeofenceLayer: border_thickness parameter must be between 0 and 1000."};
  }
  border_thickness_ = static_cast<unsigned int>(border_thick_param);

  double temp_tf_tol = 0.0;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  if (layered_costmap_->isRolling() && resize_to_fence_) {
    RCLCPP_WARN(
      logger_,
      "GeofenceLayer: resize_to_fence is not supported for rolling costmaps. Disabling.");
    resize_to_fence_ = false;
  }

  std::string polygon_str = node->declare_or_get_parameter(
    name_ + "." + "fence_polygon", std::string(""));

  if (!polygon_str.empty() && polygon_str != "[]") {
    std::vector<geometry_msgs::msg::Point> pts;
    if (nav2_costmap_2d::makeFootprintFromString(polygon_str, pts)) {
      bufferPolygon(pts);
      RCLCPP_INFO(
        logger_,
        "GeofenceLayer: Loaded geofence polygon from parameters with %zu vertices",
        pts.size());
    } else {
      throw std::runtime_error{
              "GeofenceLayer: fence_polygon parameter is malformed. "
              "Expected format: \"[[x1,y1],[x2,y2],...]\" with >= 3 vertices."};
    }
  }

  set_fence_service_ = node->create_service<nav2_msgs::srv::SetFence>(
    name_ + "/set_fence",
    std::bind(
      &GeofenceLayer::setFenceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void
GeofenceLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"GeofenceLayer: Failed to lock node"};
  }

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
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
}

void
GeofenceLayer::bufferPolygon(const std::vector<geometry_msgs::msg::Point> & pts)
{
  auto poly = std::make_shared<geometry_msgs::msg::PolygonStamped>();
  poly->header.frame_id = global_frame_;
  for (const auto & pt : pts) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(pt.x);
    p.y = static_cast<float>(pt.y);
    p.z = 0.0f;
    poly->polygon.points.push_back(p);
  }

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  polygon_buffer_ = poly;
  has_updated_data_ = true;
  setCurrent(false);
}

void
GeofenceLayer::setFenceCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetFence::Response> response)
{
  const auto & polygon = request->fence.polygon;

  if (polygon.points.empty()) {
    {
      std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
      polygon_buffer_ = std::make_shared<geometry_msgs::msg::PolygonStamped>();
      has_updated_data_ = true;
      setCurrent(false);
    }
    response->success = true;
    response->message = "Geofence cleared";
    RCLCPP_INFO(logger_, "GeofenceLayer: Fence cleared via service");
    return;
  }

  if (polygon.points.size() < 3) {
    response->success = false;
    response->message = "Geofence polygon must have at least 3 vertices";
    RCLCPP_WARN(
      logger_,
      "GeofenceLayer: Rejected polygon with %zu vertices (need >= 3)",
      polygon.points.size());
    return;
  }

  geometry_msgs::msg::PolygonStamped transformed;
  const std::string & src_frame = request->fence.header.frame_id;
  if (!src_frame.empty() && src_frame != global_frame_) {
    try {
      auto tf_stamped = tf_->lookupTransform(
        global_frame_, src_frame, tf2::TimePointZero,
        transform_tolerance_);
      tf2::doTransform(request->fence, transformed, tf_stamped);
    } catch (const tf2::TransformException & ex) {
      response->success = false;
      response->message = std::string("TF transform failed: ") + ex.what();
      RCLCPP_ERROR(logger_, "GeofenceLayer: %s", response->message.c_str());
      return;
    }
  } else {
    transformed = request->fence;
  }

  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    polygon_buffer_ = std::make_shared<geometry_msgs::msg::PolygonStamped>(transformed);
    has_updated_data_ = true;
    setCurrent(false);
  }

  response->success = true;
  response->message = "Geofence set with " +
    std::to_string(transformed.polygon.points.size()) + " vertices";
  RCLCPP_INFO(
    logger_, "GeofenceLayer: Fence updated via service with %zu vertices",
    transformed.polygon.points.size());
}

void
GeofenceLayer::processFence()
{
  matchSize();

  auto poly = polygon_buffer_;
  polygon_buffer_ = nullptr;

  if (poly) {
    if (poly->polygon.points.empty()) {
      has_fence_ = false;
      polygon_points_.clear();
      resetMaps();
      return;
    }

    polygon_points_.clear();
    for (const auto & pt : poly->polygon.points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      polygon_points_.push_back(p);
    }
    has_fence_ = true;
  }

  if (!has_fence_) {
    return;
  }

  double poly_min_x = std::numeric_limits<double>::max();
  double poly_min_y = std::numeric_limits<double>::max();
  double poly_max_x = std::numeric_limits<double>::lowest();
  double poly_max_y = std::numeric_limits<double>::lowest();

  for (const auto & p : polygon_points_) {
    poly_min_x = std::min(poly_min_x, p.x);
    poly_min_y = std::min(poly_min_y, p.y);
    poly_max_x = std::max(poly_max_x, p.x);
    poly_max_y = std::max(poly_max_y, p.y);
  }

  Costmap2D * master = layered_costmap_->getCostmap();
  double resolution = master->getResolution();

  if (resize_to_fence_) {
    double circ_r = layered_costmap_->getCircumscribedRadius();
    double padding = std::ceil((circ_r + border_thickness_ * resolution) / resolution) * resolution;

    double new_origin_x = poly_min_x - padding;
    double new_origin_y = poly_min_y - padding;
    unsigned int new_size_x = static_cast<unsigned int>(
      std::ceil((poly_max_x + padding - new_origin_x) / resolution));
    unsigned int new_size_y = static_cast<unsigned int>(
      std::ceil((poly_max_y + padding - new_origin_y) / resolution));

    constexpr double EPS = 1e-6;
    if (new_size_x != master->getSizeInCellsX() ||
      new_size_y != master->getSizeInCellsY() ||
      std::abs(master->getOriginX() - new_origin_x) >= EPS ||
      std::abs(master->getOriginY() - new_origin_y) >= EPS)
    {
      RCLCPP_INFO(
        logger_,
        "GeofenceLayer: Resizing costmap to %u x %u cells, origin (%.2f, %.2f)",
        new_size_x, new_size_y, new_origin_x, new_origin_y);
      layered_costmap_->resizeMap(
        new_size_x, new_size_y, resolution,
        new_origin_x, new_origin_y,
        true);
    }
  }

  rasterizeFence();
}

void
GeofenceLayer::rasterizeFence()
{
  resetMaps();

  unsigned int sx = getSizeInCellsX();
  unsigned int sy = getSizeInCellsY();
  if (sx == 0 || sy == 0 || polygon_points_.size() < 3) {
    return;
  }

  std::vector<MapLocation> map_polygon;
  bool any_inside = false;
  for (const auto & pt : polygon_points_) {
    MapLocation loc;
    if (worldToMap(pt.x, pt.y, loc.x, loc.y)) {
      any_inside = true;
    } else {
      double rel_x = (pt.x - getOriginX()) / getResolution();
      double rel_y = (pt.y - getOriginY()) / getResolution();
      loc.x = static_cast<unsigned int>(
        std::clamp<int>(static_cast<int>(rel_x), 0, static_cast<int>(sx - 1)));
      loc.y = static_cast<unsigned int>(
        std::clamp<int>(static_cast<int>(rel_y), 0, static_cast<int>(sy - 1)));
    }
    map_polygon.push_back(loc);
  }

  if (!any_inside) {
    RCLCPP_WARN(logger_, "GeofenceLayer: All polygon vertices are outside costmap bounds");
    double center_wx, center_wy;
    mapToWorld(sx / 2, sy / 2, center_wx, center_wy);
    if (!nav2_util::geometry_utils::isPointInsidePolygon(center_wx, center_wy, polygon_points_)) {
      std::fill(costmap_, costmap_ + sx * sy, LETHAL_OBSTACLE);
    }
    return;
  }

  std::vector<MapLocation> outline_cells;
  polygonOutlineCells(map_polygon, outline_cells);

  if (outline_cells.empty()) {
    return;
  }

  for (const auto & c : outline_cells) {
    costmap_[c.y * sx + c.x] = LETHAL_OBSTACLE;
  }

  if (border_thickness_ == 0) {
    return;
  }

  // flood-fill from map edges to find exterior cells
  std::vector<bool> is_exterior(sx * sy, false);
  std::queue<std::pair<unsigned int, unsigned int>> ext_q;

  for (unsigned int x = 0; x < sx; ++x) {
    for (unsigned int y : {0u, sy - 1}) {
      unsigned int idx = y * sx + x;
      if (costmap_[idx] != LETHAL_OBSTACLE && !is_exterior[idx]) {
        is_exterior[idx] = true;
        ext_q.push({x, y});
      }
    }
  }
  for (unsigned int y = 1; y + 1 < sy; ++y) {
    for (unsigned int x : {0u, sx - 1}) {
      unsigned int idx = y * sx + x;
      if (costmap_[idx] != LETHAL_OBSTACLE && !is_exterior[idx]) {
        is_exterior[idx] = true;
        ext_q.push({x, y});
      }
    }
  }

  static const int dx4[] = {-1, 0, 1, 0};
  static const int dy4[] = {0, -1, 0, 1};
  while (!ext_q.empty()) {
    unsigned int cx = ext_q.front().first;
    unsigned int cy = ext_q.front().second;
    ext_q.pop();
    for (int d = 0; d < 4; ++d) {
      int nx = static_cast<int>(cx) + dx4[d];
      int ny = static_cast<int>(cy) + dy4[d];
      if (nx < 0 || ny < 0 ||
        static_cast<unsigned int>(nx) >= sx ||
        static_cast<unsigned int>(ny) >= sy)
      {
        continue;
      }
      unsigned int nidx = static_cast<unsigned int>(ny) * sx +
        static_cast<unsigned int>(nx);
      if (!is_exterior[nidx] && costmap_[nidx] != LETHAL_OBSTACLE) {
        is_exterior[nidx] = true;
        ext_q.push({static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)});
      }
    }
  }

  // BFS from outline into exterior cells, up to border_thickness_ steps
  const unsigned int INF = std::numeric_limits<unsigned int>::max();
  std::vector<unsigned int> bfs_dist(sx * sy, INF);
  std::queue<MapLocation> bfs;

  static const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};

  for (const auto & c : outline_cells) {
    unsigned int idx = c.y * sx + c.x;
    if (bfs_dist[idx] == INF) {
      bfs_dist[idx] = 0;
      bfs.push(c);
    }
  }
  while (!bfs.empty()) {
    MapLocation cur = bfs.front();
    bfs.pop();
    unsigned int cur_dist = bfs_dist[cur.y * sx + cur.x];
    if (cur_dist >= border_thickness_) {
      continue;
    }
    for (int d = 0; d < 8; ++d) {
      int nx = static_cast<int>(cur.x) + dx8[d];
      int ny = static_cast<int>(cur.y) + dy8[d];
      if (nx < 0 || ny < 0 ||
        static_cast<unsigned int>(nx) >= sx ||
        static_cast<unsigned int>(ny) >= sy)
      {
        continue;
      }
      unsigned int nidx = static_cast<unsigned int>(ny) * sx +
        static_cast<unsigned int>(nx);
      if (bfs_dist[nidx] != INF || !is_exterior[nidx]) {
        continue;
      }
      bfs_dist[nidx] = cur_dist + 1;
      costmap_[nidx] = LETHAL_OBSTACLE;
      MapLocation nloc;
      nloc.x = static_cast<unsigned int>(nx);
      nloc.y = static_cast<unsigned int>(ny);
      bfs.push(nloc);
    }
  }
}

void
GeofenceLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (has_updated_data_) {
    processFence();
    has_updated_data_ = false;
    setCurrent(true);

    Costmap2D * master = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, master->getOriginX());
    *min_y = std::min(*min_y, master->getOriginY());
    *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
    *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
  }

  useExtraBounds(min_x, min_y, max_x, max_y);
}

void
GeofenceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || !has_fence_) {
    setCurrent(true);
    return;
  }
  if (layered_costmap_->isRolling()) {
    static int count = 0;
    if (++count == 10) {
      RCLCPP_WARN(logger_, "GeofenceLayer: not supported on rolling costmaps, skipping");
      count = 0;
    }
    setCurrent(true);
    return;
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  setCurrent(true);
}

rcl_interfaces::msg::SetParametersResult
GeofenceLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_name == name_ + "." + "enabled") {
      if (param.get_type() != ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enabled must be a boolean";
        return result;
      }
    } else if (param_name == name_ + "." + "resize_to_fence") {
      if (param.get_type() != ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "resize_to_fence must be a boolean";
        return result;
      }
    } else if (param_name == name_ + "." + "fence_polygon") {
      if (param.get_type() != ParameterType::PARAMETER_STRING) {
        result.successful = false;
        result.reason = "fence_polygon must be a string in footprint format";
        return result;
      }

      const std::string & poly_str = param.as_string();
      if (poly_str.empty() || poly_str == "[]") {
        continue;
      }

      std::vector<geometry_msgs::msg::Point> pts;
      if (!nav2_costmap_2d::makeFootprintFromString(poly_str, pts)) {
        result.successful = false;
        result.reason =
          "fence_polygon must be a valid polygon string, e.g. \"[[x1,y1],[x2,y2],[x3,y3]]\"";
        return result;
      }
    } else if (param_name == name_ + "." + "border_thickness") {
      if (param.get_type() != ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "border_thickness must be an integer";
        return result;
      }
      if (param.as_int() < 0 || param.as_int() > 1000) {
        result.successful = false;
        result.reason = "border_thickness must be between 0 and 1000";
        return result;
      }
    }
  }
  return result;
}

void
GeofenceLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_name == name_ + "." + "enabled") {
      enabled_ = param.as_bool();
      RCLCPP_INFO(logger_, "GeofenceLayer: %s", enabled_ ? "enabled" : "disabled");
    } else if (param_name == name_ + "." + "resize_to_fence") {
      if (layered_costmap_->isRolling()) {
        RCLCPP_WARN(
          logger_,
          "GeofenceLayer: resize_to_fence cannot be enabled for rolling costmaps. Ignoring.");
      } else {
        resize_to_fence_ = param.as_bool();
      }
    } else if (param_name == name_ + "." + "fence_polygon") {
      const std::string & poly_str = param.as_string();
      if (poly_str.empty() || poly_str == "[]") {
        bufferPolygon({});
        RCLCPP_INFO(logger_, "GeofenceLayer: Cleared geofence polygon from parameter");
      } else {
        std::vector<geometry_msgs::msg::Point> pts;
        if (nav2_costmap_2d::makeFootprintFromString(poly_str, pts)) {
          bufferPolygon(pts);
          RCLCPP_INFO(
            logger_,
            "GeofenceLayer: Updated geofence polygon from parameter with %zu vertices",
            pts.size());
        }
      }
    } else if (param_name == name_ + "." + "border_thickness") {
      border_thickness_ = static_cast<unsigned int>(param.as_int());
      has_updated_data_ = true;
      setCurrent(false);
      RCLCPP_INFO(logger_, "GeofenceLayer: Updated border_thickness to %u", border_thickness_);
    }
  }
}

}  // namespace nav2_costmap_2d
