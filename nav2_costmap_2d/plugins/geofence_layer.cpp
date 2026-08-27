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
#include <string>
#include <vector>

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

  enabled_ = node->declare_or_get_parameter(name_ + ".enabled", true);
  resize_to_fence_ = node->declare_or_get_parameter(name_ + ".resize_to_fence", true);

  // For rolling costmaps, resizing is not appropriate — warn and override.
  if (layered_costmap_->isRolling() && resize_to_fence_) {
    RCLCPP_WARN(
      logger_,
      "GeofenceLayer: resize_to_fence is not supported for rolling costmaps. Disabling.");
    resize_to_fence_ = false;
  }

  // Optional initial geofence polygon from parameter.
  // Uses the same string format as robot footprint: "[[x1,y1],[x2,y2],...]"
  std::string polygon_str = node->declare_or_get_parameter(
    name_ + ".fence_polygon", std::string(""));

  if (!polygon_str.empty()) {
    std::vector<geometry_msgs::msg::Point> pts;
    if (nav2_costmap_2d::makeFootprintFromString(polygon_str, pts)) {
      if (pts.size() < 3) {
        throw std::runtime_error{
          "GeofenceLayer: fence_polygon parameter must have >= 3 vertices."};
      }
      auto poly = std::make_shared<geometry_msgs::msg::PolygonStamped>();
      poly->header.frame_id = global_frame_;
      for (const auto & pt : pts) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(pt.x);
        p.y = static_cast<float>(pt.y);
        p.z = 0.0f;
        poly->polygon.points.push_back(p);
      }
      {
        std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
        polygon_buffer_ = poly;
      }
      polygon_updated_.store(true);
      RCLCPP_INFO(
        logger_,
        "GeofenceLayer: Loaded geofence polygon from parameters with %zu vertices",
        pts.size());
    } else {
      // Invalid parameter at init time — throw so the operator is immediately aware.
      throw std::runtime_error{
        "GeofenceLayer: fence_polygon parameter is malformed. "
        "Expected format: \"[[x1,y1],[x2,y2],...]\" with >= 3 vertices."};
    }
  }

  // Advertise the SetFence service
  set_fence_service_ = node->create_service<nav2_msgs::srv::SetFence>(
    name_ + "/set_fence",
    std::bind(
      &GeofenceLayer::setFenceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  matchSize();
  setCurrent(true);
}

void
GeofenceLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"GeofenceLayer: Failed to lock node"};
  }

  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&GeofenceLayer::validateParameterUpdatesCallback, this, std::placeholders::_1));
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(&GeofenceLayer::updateParametersCallback, this, std::placeholders::_1));
}

void
GeofenceLayer::deactivate()
{
  auto node = node_.lock();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
}

void
GeofenceLayer::reset()
{
  has_fence_ = false;
  polygon_updated_.store(false);
  polygon_x_.clear();
  polygon_y_.clear();
  outside_fence_.clear();
  mask_size_x_ = 0;
  mask_size_y_ = 0;
  mask_origin_x_ = 0.0;
  mask_origin_y_ = 0.0;
  mask_resolution_ = 0.0;
  {
    std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
    polygon_buffer_ = nullptr;
  }
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  resetMaps();
  setCurrent(true);
}

void
GeofenceLayer::matchSize()
{
  if (!layered_costmap_) {
    return;
  }
  CostmapLayer::matchSize();
}

void
GeofenceLayer::setFenceCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetFence::Response> response)
{
  const auto & polygon = request->fence.polygon;

  // Empty polygon - clear the geofence
  if (polygon.points.empty()) {
    {
      std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
      polygon_buffer_ = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    }
    polygon_updated_.store(true);
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

  // Transform to global_frame_ if needed
  geometry_msgs::msg::PolygonStamped transformed;
  const std::string & src_frame = request->fence.header.frame_id;
  if (!src_frame.empty() && src_frame != global_frame_) {
    if (!tf_) {
      response->success = false;
      response->message = "TF buffer not available; cannot transform polygon";
      RCLCPP_ERROR(logger_, "GeofenceLayer: %s", response->message.c_str());
      return;
    }
    try {
      auto tf_stamped = tf_->lookupTransform(
        global_frame_, src_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.5));
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
    std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
    polygon_buffer_ = std::make_shared<const geometry_msgs::msg::PolygonStamped>(transformed);
  }
  polygon_updated_.store(true);

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
  // Consume the buffered polygon
  std::shared_ptr<const geometry_msgs::msg::PolygonStamped> poly;
  {
    std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
    poly = polygon_buffer_;
    polygon_buffer_ = nullptr;
  }

  if (!poly) {
    return;
  }

  // Empty polygon - clear
  if (poly->polygon.points.empty()) {
    has_fence_ = false;
    polygon_x_.clear();
    polygon_y_.clear();
    outside_fence_.clear();
    mask_size_x_ = 0;
    mask_size_y_ = 0;
    return;
  }

  // Extract polygon points
  polygon_x_.clear();
  polygon_y_.clear();
  for (const auto & pt : poly->polygon.points) {
    polygon_x_.push_back(static_cast<double>(pt.x));
    polygon_y_.push_back(static_cast<double>(pt.y));
  }
  has_fence_ = true;

  // Compute bounding box of the polygon
  double poly_min_x = *std::min_element(polygon_x_.begin(), polygon_x_.end());
  double poly_min_y = *std::min_element(polygon_y_.begin(), polygon_y_.end());
  double poly_max_x = *std::max_element(polygon_x_.begin(), polygon_x_.end());
  double poly_max_y = *std::max_element(polygon_y_.begin(), polygon_y_.end());

  Costmap2D * master = layered_costmap_->getCostmap();
  double resolution = master->getResolution();

  if (resize_to_fence_) {
    // Pad by one cell beyond the circumscribed radius so the robot footprint
    // fits entirely inside the fence without clipping.
    double circ_r = layered_costmap_->getCircumscribedRadius();
    double padding = std::ceil(circ_r / resolution) * resolution;

    double new_origin_x = poly_min_x - padding;
    double new_origin_y = poly_min_y - padding;
    unsigned int new_size_x = static_cast<unsigned int>(
      std::ceil((poly_max_x + padding - new_origin_x) / resolution));
    unsigned int new_size_y = static_cast<unsigned int>(
      std::ceil((poly_max_y + padding - new_origin_y) / resolution));

    // Only resize if dimensions or origin actually changed
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
        true);  // size_locked = true
    }
  }

  // Cache the mask dimensions from the (possibly just-resized) master costmap
  mask_size_x_ = master->getSizeInCellsX();
  mask_size_y_ = master->getSizeInCellsY();
  mask_origin_x_ = master->getOriginX();
  mask_origin_y_ = master->getOriginY();
  mask_resolution_ = resolution;

  // Pre-rasterize the fence mask
  rasterizeMask();
}

void
GeofenceLayer::rasterizeMask()
{
  outside_fence_.assign(mask_size_x_ * mask_size_y_, false);
  for (unsigned int j = 0; j < mask_size_y_; ++j) {
    for (unsigned int i = 0; i < mask_size_x_; ++i) {
      double wx = mask_origin_x_ + (i + 0.5) * mask_resolution_;
      double wy = mask_origin_y_ + (j + 0.5) * mask_resolution_;
      outside_fence_[j * mask_size_x_ + i] = !isPointInPolygon(wx, wy);
    }
  }
}

bool
GeofenceLayer::isPointInPolygon(double wx, double wy) const
{
  int n = static_cast<int>(polygon_x_.size());
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon_y_[i] > wy) != (polygon_y_[j] > wy)) &&
      (wx < (polygon_x_[j] - polygon_x_[i]) * (wy - polygon_y_[i]) /
      (polygon_y_[j] - polygon_y_[i]) + polygon_x_[i]))
    {
      inside = !inside;
    }
  }
  return inside;
}

void
GeofenceLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // Process any newly received polygon (called from the costmap update thread,
  // which holds the LayeredCostmap lock — safe to call resizeMap here).
  if (polygon_updated_.load()) {
    processFence();
    polygon_updated_.store(false);
    setCurrent(true);
  }

  if (!has_fence_) {
    return;
  }

  // Expand the update window to cover the entire costmap — we rewrite all cells
  Costmap2D * master = layered_costmap_->getCostmap();
  double wx, wy;
  master->mapToWorld(0, 0, wx, wy);
  *min_x = std::min(*min_x, wx);
  *min_y = std::min(*min_y, wy);
  master->mapToWorld(master->getSizeInCellsX(), master->getSizeInCellsY(), wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
}

void
GeofenceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_fence_ || outside_fence_.empty()) {
    setCurrent(true);
    return;
  }

  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      // Bounds-check against the pre-rasterized mask dimensions
      if (i < 0 || i >= static_cast<int>(mask_size_x_) ||
        j < 0 || j >= static_cast<int>(mask_size_y_))
      {
        continue;
      }
      // Flat array lookup — O(1) per cell, no floating-point math
      if (outside_fence_[j * mask_size_x_ + i]) {
        master[j * span + i] = LETHAL_OBSTACLE;
      }
      // Inside fence: do NOT touch existing costs; let other layers set them.
    }
  }
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

    if (param_name == name_ + ".enabled") {
      if (param.get_type() != ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enabled must be a boolean";
        return result;
      }
    } else if (param_name == name_ + ".resize_to_fence") {
      if (param.get_type() != ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "resize_to_fence must be a boolean";
        return result;
      }
    } else if (param_name == name_ + ".fence_polygon") {
      if (param.get_type() != ParameterType::PARAMETER_STRING) {
        result.successful = false;
        result.reason = "fence_polygon must be a string in footprint format";
        return result;
      }
      // Validate the polygon string
      std::vector<geometry_msgs::msg::Point> pts;
      if (!nav2_costmap_2d::makeFootprintFromString(param.as_string(), pts) || pts.size() < 3) {
        result.successful = false;
        result.reason =
          "fence_polygon must be a valid polygon string with >= 3 vertices, "
          "e.g. \"[[x1,y1],[x2,y2],[x3,y3]]\"";
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
  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_name == name_ + ".enabled") {
      enabled_ = param.as_bool();
      RCLCPP_INFO(logger_, "GeofenceLayer: %s", enabled_ ? "enabled" : "disabled");
    } else if (param_name == name_ + ".resize_to_fence") {
      if (layered_costmap_->isRolling()) {
        RCLCPP_WARN(
          logger_,
          "GeofenceLayer: resize_to_fence cannot be enabled for rolling costmaps. Ignoring.");
      } else {
        resize_to_fence_ = param.as_bool();
      }
    } else if (param_name == name_ + ".fence_polygon") {
      std::vector<geometry_msgs::msg::Point> pts;
      if (nav2_costmap_2d::makeFootprintFromString(param.as_string(), pts) && pts.size() >= 3) {
        auto poly = std::make_shared<geometry_msgs::msg::PolygonStamped>();
        poly->header.frame_id = global_frame_;
        for (const auto & pt : pts) {
          geometry_msgs::msg::Point32 p;
          p.x = static_cast<float>(pt.x);
          p.y = static_cast<float>(pt.y);
          p.z = 0.0f;
          poly->polygon.points.push_back(p);
        }
        {
          std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
          polygon_buffer_ = poly;
        }
        polygon_updated_.store(true);
        RCLCPP_INFO(
          logger_,
          "GeofenceLayer: Updated geofence polygon from parameter with %zu vertices",
          pts.size());
      }
    }
  }
}

}  // namespace nav2_costmap_2d
