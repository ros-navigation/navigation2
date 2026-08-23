// Copyright (c) 2026, LonelyGuy-SE1
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

#include "nav2_costmap_2d/fence_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::FenceLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d
{

FenceLayer::FenceLayer()
{
}

FenceLayer::~FenceLayer()
{
}

void FenceLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  enabled_ = node->declare_or_get_parameter(name_ + ".enabled", true);
  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string polygon_topic = node->declare_or_get_parameter(
    name_ + ".polygon_topic", std::string("operational_zone"));
  polygon_topic = joinWithParentNamespace(polygon_topic);

  polygon_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    polygon_topic,
    std::bind(&FenceLayer::polygonCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS());

  matchSize();
  setCurrent(true);

  RCLCPP_INFO(logger_, "FenceLayer initialized, subscribing to [%s]", polygon_topic.c_str());
}

void FenceLayer::polygonCallback(
  const geometry_msgs::msg::PolygonStamped::ConstSharedPtr & msg)
{
  if (!msg || msg->polygon.points.size() < 3) {
    RCLCPP_WARN(logger_, "FenceLayer: Polygon has < 3 points, ignoring");
    return;
  }

  // Transform polygon to global_frame_ if needed
  geometry_msgs::msg::PolygonStamped transformed;
  if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) {
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "FenceLayer: Cannot transform polygon, tf_ is null");
      return;
    }
    try {
      auto transform = tf_->lookupTransform(
        global_frame_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      tf2::doTransform(*msg, transformed, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "FenceLayer: Failed to transform polygon: %s", ex.what());
      return;
    }
  } else {
    transformed = *msg;
  }

  // Buffer the polygon (consumed in updateBounds under updateMap lock)
  {
    std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
    polygon_buffer_ = std::make_shared<const geometry_msgs::msg::PolygonStamped>(transformed);
  }
  polygon_updated_.store(true);
  setCurrent(false);
}

void FenceLayer::processFence()
{
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr poly;
  {
    std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
    poly = polygon_buffer_;
    polygon_buffer_ = nullptr;
  }

  if (!poly) {
    return;
  }

  // Extract polygon points
  polygon_x_.clear();
  polygon_y_.clear();
  for (const auto & pt : poly->polygon.points) {
    polygon_x_.push_back(pt.x);
    polygon_y_.push_back(pt.y);
  }
  has_received_polygon_ = true;

  // Compute bounding box
  double min_x = *std::min_element(polygon_x_.begin(), polygon_x_.end());
  double min_y = *std::min_element(polygon_y_.begin(), polygon_y_.end());
  double max_x = *std::max_element(polygon_x_.begin(), polygon_x_.end());
  double max_y = *std::max_element(polygon_y_.begin(), polygon_y_.end());

  // Pad by circumscribed radius (rounded up to cell boundary)
  Costmap2D * master = layered_costmap_->getCostmap();
  double resolution = master->getResolution();
  double circ_r = layered_costmap_->getCircumscribedRadius();
  double padding = std::ceil(circ_r / resolution) * resolution;

  min_x -= padding;
  min_y -= padding;
  max_x += padding;
  max_y += padding;

  // Compute new grid dimensions
  unsigned int new_size_x = static_cast<unsigned int>(std::ceil((max_x - min_x) / resolution));
  unsigned int new_size_y = static_cast<unsigned int>(std::ceil((max_y - min_y) / resolution));

  // Check if layered costmap needs resizing
  if (new_size_x != master->getSizeInCellsX() ||
    new_size_y != master->getSizeInCellsY() ||
    std::abs(master->getOriginX() - min_x) >= 1e-6 ||
    std::abs(master->getOriginY() - min_y) >= 1e-6)
  {
    RCLCPP_INFO(
      logger_,
      "FenceLayer: Resizing costmap to %u X %u, origin (%.2f, %.2f)",
      new_size_x, new_size_y, min_x, min_y);

    layered_costmap_->resizeMap(
      new_size_x, new_size_y, resolution,
      min_x, min_y,
      true);  // size_locked=true
  }

  fence_size_x_ = new_size_x;
  fence_size_y_ = new_size_y;
  fence_origin_x_ = min_x;
  fence_origin_y_ = min_y;
  fence_resolution_ = resolution;

  // Pre-rasterize the fence mask
  rasterizeMask();
}

void FenceLayer::rasterizeMask()
{
  outside_fence_.assign(fence_size_x_ * fence_size_y_, false);

  for (unsigned int j = 0; j < fence_size_y_; ++j) {
    for (unsigned int i = 0; i < fence_size_x_; ++i) {
      double wx = fence_origin_x_ + (i + 0.5) * fence_resolution_;
      double wy = fence_origin_y_ + (j + 0.5) * fence_resolution_;
      outside_fence_[j * fence_size_x_ + i] = !isPointInPolygon(wx, wy);
    }
  }
}

bool FenceLayer::isPointInPolygon(double x, double y) const
{
  int n = static_cast<int>(polygon_x_.size());
  if (n < 3) {
    return false;
  }

  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon_y_[i] > y) != (polygon_y_[j] > y)) &&
      (x < (polygon_x_[j] - polygon_x_[i]) * (y - polygon_y_[i]) /
      (polygon_y_[j] - polygon_y_[i]) + polygon_x_[i]))
    {
      inside = !inside;
    }
  }
  return inside;
}

void FenceLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;

  if (!enabled_) {
    return;
  }

  // Check if new polygon is available
  if (polygon_updated_.load()) {
    processFence();  // Runs under updateMap's lock — safe to call resizeMap
    polygon_updated_.store(false);
    setCurrent(true);
  }

  if (!has_received_polygon_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  // Report entire costmap as needing update (we rewrite all cells)
  Costmap2D * master = layered_costmap_->getCostmap();
  double wx, wy;
  master->mapToWorld(0, 0, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  master->mapToWorld(master->getSizeInCellsX(), master->getSizeInCellsY(), wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);
}

void FenceLayer::updateCosts(
  Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_received_polygon_) {
    return;
  }

  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      // Bounds check against pre-rasterized mask
      if (i < 0 || i >= static_cast<int>(fence_size_x_) ||
        j < 0 || j >= static_cast<int>(fence_size_y_))
      {
        continue;
      }

      if (outside_fence_[j * fence_size_x_ + i]) {
        master[j * span + i] = LETHAL_OBSTACLE;
      }
      // Inside fence: do NOT touch. Let other layers handle costs.
    }
  }
}

void FenceLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(&FenceLayer::updateParametersCallback, this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FenceLayer::validateParameterUpdatesCallback, this, std::placeholders::_1));
}

void FenceLayer::deactivate()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  if (post_set_params_handler_) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
    post_set_params_handler_.reset();
  }
  if (on_set_params_handler_) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
    on_set_params_handler_.reset();
  }
}

void FenceLayer::reset()
{
  has_received_polygon_ = false;
  polygon_updated_.store(false);
  polygon_x_.clear();
  polygon_y_.clear();
  outside_fence_.clear();
  fence_size_x_ = 0;
  fence_size_y_ = 0;
  fence_origin_x_ = 0.0;
  fence_origin_y_ = 0.0;
  fence_resolution_ = 0.0;

  {
    std::lock_guard<std::mutex> lock(polygon_buffer_mutex_);
    polygon_buffer_ = nullptr;
  }

  // Reset internal costmap
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  resetMaps();
  setCurrent(true);
}

void FenceLayer::matchSize()
{
  if (!layered_costmap_) {
    return;
  }
  CostmapLayer::matchSize();
}

rcl_interfaces::msg::SetParametersResult
FenceLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == name_ + ".enabled") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enabled must be a boolean";
        return result;
      }
    }
  }
  return result;
}

void FenceLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == name_ + ".enabled") {
      enabled_ = param.as_bool();
    }
  }
}

}  // namespace nav2_costmap_2d
