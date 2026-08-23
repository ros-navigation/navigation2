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

#include "nav2_costmap_2d/fence_layer.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::FenceLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

FenceLayer::FenceLayer()
{
}

FenceLayer::~FenceLayer()
{
}

void
FenceLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);

  // Parse optional fence polygon from parameters as a flat list [x1,y1,x2,y2,...]
  std::vector<double> fence_param;
  fence_param = node->declare_or_get_parameter(
    name_ + "." + "fence_polygon", std::vector<double>{});

  if (!fence_param.empty()) {
    std::vector<geometry_msgs::msg::Point32> polygon;
    if (parsePolygonParameter(fence_param, polygon)) {
      std::lock_guard<std::mutex> lock(fence_mutex_);
      fence_polygon_ = polygon;
      has_fence_ = true;
      fence_changed_ = true;
      RCLCPP_INFO(
        logger_,
        "FenceLayer: Loaded fence polygon from parameters with %zu vertices",
        fence_polygon_.size());
    } else {
      RCLCPP_ERROR(
        logger_,
        "FenceLayer: Invalid fence_polygon parameter. "
        "Must be a flat list of doubles [x1,y1,x2,y2,...] with >= 3 vertices.");
    }
  }

  // Advertise the SetFence service
  set_fence_service_ = node->create_service<nav2_msgs::srv::SetFence>(
    name_ + "/set_fence",
    std::bind(
      &FenceLayer::setFenceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void
FenceLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const auto & parameter : parameters) {
          const auto & param_name = parameter.get_name();
          if (param_name == name_ + ".enabled") {
            enabled_ = parameter.as_bool();
            fence_changed_ = true;
            RCLCPP_INFO(
              logger_, "FenceLayer: %s",
              enabled_ ? "enabled" : "disabled");
          }
        }
      },
      std::placeholders::_1));
}

void
FenceLayer::deactivate()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
}

void
FenceLayer::reset()
{
  fence_changed_ = true;
  setCurrent(false);
}

void
FenceLayer::setFenceCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
  std::shared_ptr<nav2_msgs::srv::SetFence::Response> response)
{
  const auto & polygon = request->fence.polygon;

  if (polygon.points.empty()) {
    // Empty polygon clears the fence
    std::lock_guard<std::mutex> lock(fence_mutex_);
    fence_polygon_.clear();
    has_fence_ = false;
    fence_changed_ = true;
    response->success = true;
    response->message = "Fence cleared";
    RCLCPP_INFO(logger_, "FenceLayer: Fence cleared via service");
    return;
  }

  if (polygon.points.size() < 3) {
    response->success = false;
    response->message = "Fence polygon must have at least 3 vertices";
    RCLCPP_WARN(
      logger_,
      "FenceLayer: Rejected fence with %zu vertices (need >= 3)",
      polygon.points.size());
    return;
  }

  // TODO(future): If request->fence.header.frame_id differs from global_frame_,
  // transform the polygon vertices. For now, we assume the polygon is in the
  // global frame.
  if (!request->fence.header.frame_id.empty() &&
    request->fence.header.frame_id != global_frame_)
  {
    RCLCPP_WARN(
      logger_,
      "FenceLayer: Fence polygon frame '%s' differs from global frame '%s'. "
      "Assuming polygon is already in the global frame.",
      request->fence.header.frame_id.c_str(), global_frame_.c_str());
  }

  {
    std::lock_guard<std::mutex> lock(fence_mutex_);
    fence_polygon_.assign(polygon.points.begin(), polygon.points.end());
    has_fence_ = true;
    fence_changed_ = true;
  }

  response->success = true;
  response->message = "Fence set with " +
    std::to_string(polygon.points.size()) + " vertices";
  RCLCPP_INFO(
    logger_,
    "FenceLayer: Fence updated via service with %zu vertices",
    polygon.points.size());
}

void
FenceLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(fence_mutex_);
  if (!has_fence_) {
    return;
  }

  // When the fence is active, we need to update the entire costmap since
  // we mark cells outside the polygon as lethal on every cycle.
  // Only expand bounds when the fence has changed to avoid unnecessary work.
  if (fence_changed_) {
    Costmap2D * master = layered_costmap_->getCostmap();
    double wx0, wy0, wx1, wy1;
    master->mapToWorld(0, 0, wx0, wy0);
    master->mapToWorld(
      master->getSizeInCellsX(), master->getSizeInCellsY(), wx1, wy1);

    *min_x = std::min(*min_x, wx0);
    *min_y = std::min(*min_y, wy0);
    *max_x = std::max(*max_x, wx1);
    *max_y = std::max(*max_y, wy1);

    fence_changed_ = false;
  }
}

void
FenceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(fence_mutex_);
  if (!has_fence_ || fence_polygon_.empty()) {
    setCurrent(true);
    return;
  }

  // For each cell in the update window, check if it is inside the fence.
  // If outside, mark it as LETHAL_OBSTACLE.
  double wx, wy;
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      if (!isInsideFence(wx, wy)) {
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j),
          LETHAL_OBSTACLE);
      }
    }
  }

  setCurrent(true);
}

bool
FenceLayer::isInsideFence(double wx, double wy) const
{
  // Ray-casting (crossing number) algorithm for point-in-polygon.
  // Cast a ray from (wx, wy) in the +x direction and count edge crossings.
  const size_t n = fence_polygon_.size();
  if (n < 3) {
    return true;  // Degenerate polygon — treat as "no fence"
  }

  bool inside = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = fence_polygon_[i].x;
    const double yi = fence_polygon_[i].y;
    const double xj = fence_polygon_[j].x;
    const double yj = fence_polygon_[j].y;

    // Check if the ray crosses this edge
    if (((yi > wy) != (yj > wy)) &&
      (wx < (xj - xi) * (wy - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

bool
FenceLayer::parsePolygonParameter(
  const std::vector<double> & raw,
  std::vector<geometry_msgs::msg::Point32> & polygon) const
{
  if (raw.size() < 6 || raw.size() % 2 != 0) {
    return false;
  }

  polygon.clear();
  polygon.reserve(raw.size() / 2);
  for (size_t i = 0; i < raw.size(); i += 2) {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(raw[i]);
    pt.y = static_cast<float>(raw[i + 1]);
    pt.z = 0.0f;
    polygon.push_back(pt);
  }
  return true;
}

}  // namespace nav2_costmap_2d
