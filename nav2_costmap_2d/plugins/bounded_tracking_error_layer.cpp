// Copyright (c) 2025 Berkan Tali
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

#include "nav2_costmap_2d/bounded_tracking_error_layer.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BoundedTrackingErrorLayer, nav2_costmap_2d::Layer)

using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

void
BoundedTrackingErrorLayer::onInitialize()
{
  getParameters();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    joinWithParentNamespace(path_topic_),
    std::bind(&BoundedTrackingErrorLayer::pathCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS());

  // Seed resolution and frame eagerly; matchSize() will keep them current on subsequent resizes.
  if (layered_costmap_) {
    resolution_ = layered_costmap_->getCostmap()->getResolution();
    costmap_frame_ = layered_costmap_->getGlobalFrameID();
  }

  current_ = true;
  RCLCPP_INFO(
    logger_,
    "BoundedTrackingErrorLayer initialized in %s mode",
    cost_write_mode_ == 0 ? "corridor" : cost_write_mode_ == 1 ? "fill (safe)" : "fill (overwrite)");
}

void
BoundedTrackingErrorLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &BoundedTrackingErrorLayer::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &BoundedTrackingErrorLayer::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void
BoundedTrackingErrorLayer::deactivate()
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
  path_sub_.reset();
}

void
BoundedTrackingErrorLayer::reset()
{
  resetState();
  current_ = false;
}

void
BoundedTrackingErrorLayer::matchSize()
{
  resolution_ = layered_costmap_->getCostmap()->getResolution();
  costmap_frame_ = layered_costmap_->getGlobalFrameID();
  const auto * costmap = layered_costmap_->getCostmap();
  corridor_interior_mask_.assign(
    costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), 0);
  prev_fill_min_i_ = -1;
  prev_fill_min_j_ = -1;
  prev_fill_max_i_ = -1;
  prev_fill_max_j_ = -1;
}

void
BoundedTrackingErrorLayer::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!last_path_ptr_ || msg->header.stamp != last_path_ptr_->header.stamp) {
    current_path_index_.store(0);
  }
  last_path_ptr_ = msg;
}

void
BoundedTrackingErrorLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  const double margin = look_ahead_ + corridor_width_ * 0.5 + wall_thickness_ * resolution_;

  *min_x = std::min(*min_x, robot_x - margin);
  *max_x = std::max(*max_x, robot_x + margin);
  *min_y = std::min(*min_y, robot_y - margin);
  *max_y = std::max(*max_y, robot_y + margin);
}

void
BoundedTrackingErrorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  [[maybe_unused]] int min_i, [[maybe_unused]] int min_j,
  [[maybe_unused]] int max_i, [[maybe_unused]] int max_j)
{
  if (!enabled_) {
    return;
  }

  if (resolution_ <= 0.0) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Resolution is not initialized (%.4f), skipping corridor update", resolution_);
    return;
  }

  nav_msgs::msg::Path::ConstSharedPtr path_ptr;
  size_t path_index;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    path_ptr = last_path_ptr_;
    path_index = current_path_index_.load();

    if (path_ptr && !path_ptr->poses.empty()) {
      const auto age = (clock_->now() - rclcpp::Time(path_ptr->header.stamp)).seconds();
      if (age > 5.0) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "Path is %.2f seconds old, clearing corridor state — waiting for new plan", age);
        last_path_ptr_.reset();
        current_path_index_.store(0);
        prev_fill_min_i_ = -1;
        prev_fill_min_j_ = -1;
        prev_fill_max_i_ = -1;
        prev_fill_max_j_ = -1;
        return;
      }
    }
  }

  if (!path_ptr || path_ptr->poses.empty()) {
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;

  const double tf_tol = tf2::durationToSec(transform_tolerance_);

  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, costmap_frame_, robot_base_frame_, tf_tol))
  {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      1000,
      "Failed to get robot pose in %s, skipping corridor update",
      costmap_frame_.c_str());
    return;
  }

  const nav_msgs::msg::Path * full_transformed_ptr = nullptr;
  if (path_ptr->header.frame_id == costmap_frame_) {
    full_transformed_ptr = path_ptr.get();
  } else {
    if (!nav2_util::transformPathInTargetFrame(
        *path_ptr, full_transformed_path_buffer_, *tf_, costmap_frame_, tf_tol))
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Failed to transform path from '%s' to '%s', skipping corridor update",
        path_ptr->header.frame_id.c_str(), costmap_frame_.c_str());
      return;
    }
    full_transformed_ptr = &full_transformed_path_buffer_;
  }

  if (path_index >= full_transformed_ptr->poses.size()) {
    path_index = 0;
    current_path_index_.store(0);
  }

  const auto search_result = nav2_util::distance_from_path(
    *full_transformed_ptr,
    robot_pose.pose,
    path_index,
    look_ahead_);
  current_path_index_.store(search_result.closest_segment_index);

  getPathSegment(*full_transformed_ptr, search_result.closest_segment_index, segment_buffer_);

  const size_t min_poses = (step_size_ * 2) + 1;
  if (segment_buffer_.poses.size() < min_poses) {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 2000,
      "Segment too small (%zu poses), need at least %zu — expected near end of path",
      segment_buffer_.poses.size(), min_poses);
    return;
  }

  if (cost_write_mode_ >= 1) {
    if (corridor_interior_mask_.size() !=
      master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY())
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "Corridor interior mask size mismatch, skipping fill update — call matchSize()");
      return;
    }
    applyFillOutsideCorridor(master_grid, robot_pose, *full_transformed_ptr);
  } else {
    getWallPolygons(segment_buffer_, walls_buffer_);
    drawCorridorWalls(master_grid, walls_buffer_.left_inner, walls_buffer_.left_outer);
    drawCorridorWalls(master_grid, walls_buffer_.right_inner, walls_buffer_.right_outer);
  }
}

void
BoundedTrackingErrorLayer::getPathSegment(
  const nav_msgs::msg::Path & path,
  const size_t path_index,
  nav_msgs::msg::Path & segment)
{
  segment.poses.clear();

  if (path.poses.empty() || path_index >= path.poses.size()) {
    return;
  }

  size_t end_index = path_index;
  double dist_traversed = 0.0;

  for (size_t i = path_index; i < path.poses.size() - 1; i++) {
    dist_traversed += nav2_util::geometry_utils::euclidean_distance(
      path.poses[i], path.poses[i + 1]);

    end_index = i + 1;
    if (dist_traversed >= look_ahead_) {
      break;
    }
  }

  if (path_index < end_index && end_index < path.poses.size()) {
    segment.header = path.header;
    segment.poses.assign(
      path.poses.begin() + path_index,
      path.poses.begin() + end_index + 1);
  } else if (path_index == path.poses.size() - 1) {
    segment.header = path.header;
    segment.poses.push_back(path.poses[path_index]);
  }
}

void
BoundedTrackingErrorLayer::getWallPolygons(
  const nav_msgs::msg::Path & segment,
  WallPolygons & walls)
{
  if (segment.poses.empty() || step_size_ == 0) {
    walls.clearAndReserve(0);
    return;
  }

  const double inner_offset = corridor_width_ * 0.5;
  const double outer_offset = inner_offset + (wall_thickness_ * resolution_);

  const size_t estimated_points = (segment.poses.size() / step_size_) + 1;
  walls.clearAndReserve(estimated_points);

  const size_t last_index = segment.poses.size() - 1;

  for (size_t current_index = 0; current_index < segment.poses.size(); ) {
    const auto & current_pose = segment.poses[current_index];
    const double px = current_pose.pose.position.x;
    const double py = current_pose.pose.position.y;

    size_t next_index = current_index + step_size_;
    if (next_index >= segment.poses.size()) {
      next_index = last_index;
      if (next_index == current_index) {
        break;
      }
    }

    const auto & next_pose = segment.poses[next_index];
    const double dx = next_pose.pose.position.x - px;
    const double dy = next_pose.pose.position.y - py;

    const double norm = std::hypot(dx, dy);
    if (norm < resolution_) {
      RCLCPP_DEBUG_THROTTLE(
        logger_, *clock_, 5000,
        "Skipping zero-length path segment at index %zu: norm %.4f < resolution %.4f",
        current_index, norm, resolution_);
      if (current_index >= last_index) {
        break;
      }
      current_index = std::min(current_index + step_size_, last_index);
      continue;
    }

    // Unit perpendicular to the path direction (rotate 90 CCW): (-dy, dx).
    // In an X-forward path frame this points to the left side of travel.
    const double inv_norm = 1.0 / norm;
    const double perp_x = -dy * inv_norm;
    const double perp_y = dx * inv_norm;

    walls.left_inner.push_back(
      {px + perp_x * inner_offset, py + perp_y * inner_offset});
    walls.right_inner.push_back(
      {px - perp_x * inner_offset, py - perp_y * inner_offset});
    walls.left_outer.push_back(
      {px + perp_x * outer_offset, py + perp_y * outer_offset});
    walls.right_outer.push_back(
      {px - perp_x * outer_offset, py - perp_y * outer_offset});

    if (current_index >= last_index) {
      break;
    }
    current_index = std::min(current_index + step_size_, last_index);
  }
}

void
BoundedTrackingErrorLayer::applyFillOutsideCorridor(
  nav2_costmap_2d::Costmap2D & master_grid,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & full_path)
{
  const double fill_radius = look_ahead_ + corridor_width_ * 0.5 + wall_thickness_ * resolution_;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  int cell_min_x, cell_min_y, cell_max_x, cell_max_y;
  master_grid.worldToMapEnforceBounds(rx - fill_radius, ry - fill_radius, cell_min_x, cell_min_y);
  master_grid.worldToMapEnforceBounds(rx + fill_radius, ry + fill_radius, cell_max_x, cell_max_y);

  const unsigned int fill_size_x = master_grid.getSizeInCellsX();
  const unsigned int fill_size_y = master_grid.getSizeInCellsY();

  int fill_min_i = std::max(cell_min_x, 0);
  int fill_min_j = std::max(cell_min_y, 0);
  int fill_max_i = std::min(cell_max_x, static_cast<int>(fill_size_x) - 1);
  int fill_max_j = std::min(cell_max_y, static_cast<int>(fill_size_y) - 1);

  const int reset_min_i = (prev_fill_min_i_ < 0) ? fill_min_i : std::min(fill_min_i,
      prev_fill_min_i_);
  const int reset_min_j = (prev_fill_min_j_ < 0) ? fill_min_j : std::min(fill_min_j,
      prev_fill_min_j_);
  const int reset_max_i = (prev_fill_max_i_ < 0) ? fill_max_i : std::max(fill_max_i,
      prev_fill_max_i_);
  const int reset_max_j = (prev_fill_max_j_ < 0) ? fill_max_j : std::max(fill_max_j,
      prev_fill_max_j_);

  resetCorridorMask(
    master_grid.getSizeInCellsX(),
    reset_min_i, reset_min_j,
    reset_max_i, reset_max_j);

  prev_fill_min_i_ = fill_min_i;
  prev_fill_min_j_ = fill_min_j;
  prev_fill_max_i_ = fill_max_i;
  prev_fill_max_j_ = fill_max_j;

  const double r_cells = (corridor_width_ * 0.5) / resolution_;
  const int r_cells_sq = static_cast<int>(std::llround(r_cells * r_cells));

  // extra_poses extends wall polygon generation beyond the bbox boundary to cover
  // geometric gaps at sub-segment exit points on diagonal paths.
  const size_t extra_poses = static_cast<size_t>(
    std::ceil(corridor_width_ / resolution_)) + step_size_;

  buildCorridorMask(
    master_grid, full_path,
    fill_min_i, fill_min_j,
    fill_max_i, fill_max_j,
    extra_poses);

  // Guarantee the robot's own cell is always interior regardless of path position
  unsigned int robot_cx, robot_cy;
  if (master_grid.worldToMap(
      robot_pose.pose.position.x, robot_pose.pose.position.y, robot_cx, robot_cy))
  {
    markCircleAsInterior(master_grid, static_cast<int>(robot_cx), static_cast<int>(robot_cy),
      r_cells_sq);
  }

  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 2000,
    "Fill bbox: [%d,%d] to [%d,%d], extra_poses=%zu",
    fill_min_i, fill_min_j, fill_max_i, fill_max_j,
    extra_poses);

  fillOutsideCorridor(master_grid, fill_min_i, fill_min_j, fill_max_i, fill_max_j);
}

void
BoundedTrackingErrorLayer::drawCorridorWalls(
  nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<std::array<double, 2>> & inner_points,
  const std::vector<std::array<double, 2>> & outer_points)
{
  if (inner_points.size() < 2 || outer_points.size() < 2) {
    RCLCPP_DEBUG(
      logger_,
      "Skipping corridor wall draw: inner=%zu, outer=%zu points (need >= 2 each)",
      inner_points.size(), outer_points.size());
    return;
  }

  const size_t num_segments = std::min(inner_points.size(), outer_points.size()) - 1;

  for (size_t i = 0; i < num_segments; ++i) {
    CellPoint inner0, inner1, outer0, outer1;

    const bool inner0_valid = worldToCell(
      master_grid, inner_points[i][0], inner_points[i][1], inner0);
    const bool inner1_valid = worldToCell(
      master_grid, inner_points[i + 1][0], inner_points[i + 1][1], inner1);
    if (!inner0_valid && !inner1_valid) {
      continue;
    }

    worldToCell(master_grid, outer_points[i][0], outer_points[i][1], outer0);
    worldToCell(master_grid, outer_points[i + 1][0], outer_points[i + 1][1], outer1);

    fillCorridorQuad(master_grid, inner0, inner1, outer0, outer1);
  }
}

void
BoundedTrackingErrorLayer::saveCorridorInterior(
  nav2_costmap_2d::Costmap2D & master_grid,
  const WallPolygons & walls)
{
  if (walls.left_inner.size() < 2 || walls.right_inner.size() < 2) {
    RCLCPP_DEBUG(
      logger_,
      "Skipping corridor interior save: left_inner=%zu, right_inner=%zu points (need >= 2 each)",
      walls.left_inner.size(), walls.right_inner.size());
    return;
  }

  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  const size_t num_segments = std::min(walls.left_inner.size(), walls.right_inner.size()) - 1;

  for (size_t i = 0; i < num_segments; ++i) {
    CellPoint left0, left1, right0, right1;

    const bool l0 = worldToCell(
      master_grid, walls.left_inner[i][0], walls.left_inner[i][1], left0);
    const bool l1 = worldToCell(
      master_grid, walls.left_inner[i + 1][0], walls.left_inner[i + 1][1], left1);
    const bool r0 = worldToCell(
      master_grid, walls.right_inner[i][0], walls.right_inner[i][1], right0);
    const bool r1 = worldToCell(
      master_grid, walls.right_inner[i + 1][0], walls.right_inner[i + 1][1], right1);

    if (!l0 && !l1 && !r0 && !r1) {
      continue;
    }

    const int y_min = std::min({left0.y, left1.y, right0.y, right1.y});
    const int y_max = std::max({left0.y, left1.y, right0.y, right1.y});
    const int clamped_y_min = std::max(y_min, 0);
    const int clamped_y_max = std::min(y_max, static_cast<int>(size_y) - 1);

    if (clamped_y_min > clamped_y_max) {
      continue;
    }

    const int height = clamped_y_max - clamped_y_min + 1;
    span_x_min_buffer_.assign(height, std::numeric_limits<int>::max());
    span_x_max_buffer_.assign(height, std::numeric_limits<int>::min());
    traceQuad(left0, left1, right1, right0, clamped_y_min, height);

    for (int buffer_idx = 0; buffer_idx < height; ++buffer_idx) {
      const int y = clamped_y_min + buffer_idx;
      const int x_min = span_x_min_buffer_[buffer_idx];
      const int x_max = span_x_max_buffer_[buffer_idx];

      if (x_min > x_max) {
        continue;
      }

      const int x_start = std::max(x_min, 0);
      const int x_end = std::min(x_max, static_cast<int>(size_x) - 1);

      for (int x = x_start; x <= x_end; ++x) {
        const unsigned int flat_idx =
          static_cast<unsigned int>(y) * size_x + static_cast<unsigned int>(x);
        corridor_interior_mask_[flat_idx] = 1;
      }
    }
  }
}

void
BoundedTrackingErrorLayer::markCircleAsInterior(
  nav2_costmap_2d::Costmap2D & master_grid,
  int cx, int cy, int r_sq)
{
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();
  const int r = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(r_sq))));

  const int x_start = std::max(cx - r, 0);
  const int x_end = std::min(cx + r, static_cast<int>(size_x) - 1);
  const int y_start = std::max(cy - r, 0);
  const int y_end = std::min(cy + r, static_cast<int>(size_y) - 1);

  for (int y = y_start; y <= y_end; ++y) {
    const int dy = y - cy;
    for (int x = x_start; x <= x_end; ++x) {
      const int dx = x - cx;
      if (dx * dx + dy * dy <= r_sq) {
        const unsigned int flat_idx =
          static_cast<unsigned int>(y) * size_x + static_cast<unsigned int>(x);
        corridor_interior_mask_[flat_idx] = 1;
      }
    }
  }
}

void
BoundedTrackingErrorLayer::fillOutsideCorridor(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  unsigned char * costmap = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  const int x_start = std::max(min_i, 0);
  const int x_end = std::min(max_i, static_cast<int>(size_x) - 1);
  const int y_start = std::max(min_j, 0);
  const int y_end = std::min(max_j, static_cast<int>(size_y) - 1);

  for (int y = y_start; y <= y_end; ++y) {
    for (int x = x_start; x <= x_end; ++x) {
      const unsigned int flat_idx = static_cast<unsigned int>(y) * size_x +
        static_cast<unsigned int>(x);
      if (corridor_interior_mask_[flat_idx]) {
        continue;
      }
      if (cost_write_mode_ == 2) {
        costmap[flat_idx] = corridor_cost_;
      } else {
        costmap[flat_idx] = std::max(costmap[flat_idx], corridor_cost_);
      }
    }
  }
}

void
BoundedTrackingErrorLayer::resetState()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_path_ptr_.reset();
  current_path_index_.store(0);
  prev_fill_min_i_ = -1;
  prev_fill_min_j_ = -1;
  prev_fill_max_i_ = -1;
  prev_fill_max_j_ = -1;
}

void
BoundedTrackingErrorLayer::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  path_topic_ = node->declare_or_get_parameter(
    name_ + "." + "path_topic", std::string("plan"));

  const int step_param = node->declare_or_get_parameter(name_ + "." + "step", 10);
  if (step_param <= 0) {
    throw std::runtime_error{"step must be greater than zero"};
  }
  step_size_ = static_cast<size_t>(step_param);

  look_ahead_ = node->declare_or_get_parameter(name_ + "." + "look_ahead", 2.5);
  if (look_ahead_ <= 0.0) {
    throw std::runtime_error{"look_ahead must be positive"};
  }

  corridor_width_ = node->declare_or_get_parameter(name_ + "." + "corridor_width", 2.0);
  if (corridor_width_ <= 0.0) {
    throw std::runtime_error{"corridor_width must be positive"};
  }

  wall_thickness_ = node->declare_or_get_parameter(name_ + "." + "wall_thickness", 1);
  if (wall_thickness_ <= 0) {
    throw std::runtime_error{"wall_thickness must be greater than zero"};
  }

  int corridor_cost_param = node->declare_or_get_parameter(name_ + "." + "corridor_cost", 190);
  if (corridor_cost_param <= 0 || corridor_cost_param > 254) {
    throw std::runtime_error{"corridor_cost must be between 1 and 254"};
  }
  corridor_cost_ = static_cast<unsigned char>(corridor_cost_param);

  int cost_write_mode_param = node->declare_or_get_parameter(
      name_ + "." + "cost_write_mode", 0);
  if (cost_write_mode_param < 0 || cost_write_mode_param > 2) {
    throw std::runtime_error{
            "cost_write_mode must be 0 (max), 1 (overwrite walls), or 2 (overwrite all)"};
  }
  cost_write_mode_ = cost_write_mode_param;

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);

  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  node->get_parameter("robot_base_frame", robot_base_frame_);
}

bool
BoundedTrackingErrorLayer::worldToCell(
  const nav2_costmap_2d::Costmap2D & master_grid,
  double wx, double wy,
  CellPoint & out) const
{
  unsigned int ux, uy;
  if (master_grid.worldToMap(wx, wy, ux, uy)) {
    out = {static_cast<int>(ux), static_cast<int>(uy)};
    return true;
  }
  // Clamp to costmap boundary so quads straddling the edge are still drawn.
  int cx, cy;
  master_grid.worldToMapNoBounds(wx, wy, cx, cy);
  out = {
    std::clamp(cx, 0, static_cast<int>(master_grid.getSizeInCellsX()) - 1),
    std::clamp(cy, 0, static_cast<int>(master_grid.getSizeInCellsY()) - 1)
  };
  return false;
}

void
BoundedTrackingErrorLayer::traceQuad(
  CellPoint p0, CellPoint p1, CellPoint p2, CellPoint p3,
  int clamped_y_min, int height)
{
  auto trace = [&](CellPoint a, CellPoint b) {
      nav2_util::LineIterator line(a.x, a.y, b.x, b.y);
      for (; line.isValid(); line.advance()) {
        const int x = line.getX();
        const int y = line.getY();
        const int buffer_idx = y - clamped_y_min;
        if (buffer_idx >= 0 && buffer_idx < height) {
          span_x_min_buffer_[buffer_idx] = std::min(span_x_min_buffer_[buffer_idx], x);
          span_x_max_buffer_[buffer_idx] = std::max(span_x_max_buffer_[buffer_idx], x);
        }
      }
    };

  trace(p0, p1);
  trace(p1, p2);
  trace(p2, p3);
  trace(p3, p0);
}

void
BoundedTrackingErrorLayer::fillCorridorQuad(
  nav2_costmap_2d::Costmap2D & master_grid,
  CellPoint inner0,
  CellPoint inner1,
  CellPoint outer0,
  CellPoint outer1)
{
  unsigned char * costmap = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  const int y_min = std::min({inner0.y, inner1.y, outer0.y, outer1.y});
  const int y_max = std::max({inner0.y, inner1.y, outer0.y, outer1.y});

  const int clamped_y_min = std::max(y_min, 0);
  const int clamped_y_max = std::min(y_max, static_cast<int>(size_y) - 1);

  if (clamped_y_min > clamped_y_max) {
    return;
  }

  const int height = clamped_y_max - clamped_y_min + 1;

  span_x_min_buffer_.assign(height, std::numeric_limits<int>::max());
  span_x_max_buffer_.assign(height, std::numeric_limits<int>::min());

  traceQuad(inner0, inner1, outer1, outer0, clamped_y_min, height);

  for (int buffer_idx = 0; buffer_idx < height; ++buffer_idx) {
    const int y = clamped_y_min + buffer_idx;
    const int x_min = span_x_min_buffer_[buffer_idx];
    const int x_max = span_x_max_buffer_[buffer_idx];

    if (x_min > x_max) {
      continue;
    }

    // Clamp to costmap bounds before writing.
    const int x_start = std::max(x_min, 0);
    const int x_end = std::min(x_max, static_cast<int>(size_x) - 1);

    for (int x = x_start; x <= x_end; ++x) {
      costmap[y * size_x + x] = std::max(costmap[y * size_x + x], corridor_cost_);
    }
  }
}

void
BoundedTrackingErrorLayer::resetCorridorMask(
  unsigned int size_x,
  int reset_min_i, int reset_min_j,
  int reset_max_i, int reset_max_j)
{
  for (int y = reset_min_j; y <= reset_max_j; ++y) {
    std::fill(
      corridor_interior_mask_.begin() + y * size_x + reset_min_i,
      corridor_interior_mask_.begin() + y * size_x + reset_max_i + 1,
      0);
  }
}

void
BoundedTrackingErrorLayer::buildCorridorMask(
  nav2_costmap_2d::Costmap2D & master_grid,
  const nav_msgs::msg::Path & full_path,
  int fill_min_i, int fill_min_j,
  int fill_max_i, int fill_max_j,
  size_t extra_poses)
{
  nav_msgs::msg::Path sub_segment;
  sub_segment.header = full_path.header;

  for (size_t i = 0; i < full_path.poses.size(); ++i) {
    const auto & pose = full_path.poses[i];
    unsigned int mx, my;
    const bool in_area = master_grid.worldToMap(
      pose.pose.position.x, pose.pose.position.y, mx, my) &&
      static_cast<int>(mx) >= fill_min_i &&
      static_cast<int>(mx) <= fill_max_i &&
      static_cast<int>(my) >= fill_min_j &&
      static_cast<int>(my) <= fill_max_j;

    if (in_area) {
      sub_segment.poses.push_back(pose);
    } else {
      for (size_t j = i; j < std::min(i + extra_poses, full_path.poses.size()); ++j) {
        sub_segment.poses.push_back(full_path.poses[j]);
      }
      flushSubSegment(
        master_grid, sub_segment,
        fill_min_i, fill_min_j,
        fill_max_i, fill_max_j,
        extra_poses);
    }
  }
  flushSubSegment(
    master_grid, sub_segment,
    fill_min_i, fill_min_j,
    fill_max_i, fill_max_j,
    extra_poses);
}

void
BoundedTrackingErrorLayer::flushSubSegment(
  nav2_costmap_2d::Costmap2D & master_grid,
  nav_msgs::msg::Path & sub_segment,
  int fill_min_i, int fill_min_j,
  int fill_max_i, int fill_max_j,
  size_t extra_poses)
{
  if (sub_segment.poses.size() < 2) {
    sub_segment.poses.clear();
    return;
  }

  nav_msgs::msg::Path bbox_segment;
  bbox_segment.header = sub_segment.header;

  auto flush_bbox_segment = [&]() {
      if (bbox_segment.poses.size() >= 2) {
        WallPolygons fill_walls;
        getWallPolygons(bbox_segment, fill_walls);
        saveCorridorInterior(master_grid, fill_walls);
      }
      bbox_segment.poses.clear();
    };

  for (const auto & p : sub_segment.poses) {
    unsigned int mx, my;
    const bool in_margin = master_grid.worldToMap(
      p.pose.position.x, p.pose.position.y, mx, my) &&
      static_cast<int>(mx) >= fill_min_i - static_cast<int>(extra_poses) &&
      static_cast<int>(mx) <= fill_max_i + static_cast<int>(extra_poses) &&
      static_cast<int>(my) >= fill_min_j - static_cast<int>(extra_poses) &&
      static_cast<int>(my) <= fill_max_j + static_cast<int>(extra_poses);

    if (in_margin) {
      bbox_segment.poses.push_back(p);
    } else {
      flush_bbox_segment();
    }
  }
  flush_bbox_segment();

  sub_segment.poses.clear();
}

rcl_interfaces::msg::SetParametersResult
BoundedTrackingErrorLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "look_ahead") {
        const double new_value = parameter.as_double();
        if (new_value <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %f, "
            "it should be > 0. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason = "look_ahead must be positive";
        }
      } else if (param_name == name_ + "." + "corridor_width") {
        const double new_value = parameter.as_double();
        if (new_value <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %f, "
            "it should be > 0. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason = "corridor_width must be positive";
        }
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "step") {
        const int new_value = parameter.as_int();
        if (new_value <= 0) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %d, "
            "it should be > 0. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason = "step must be greater than zero";
        }
      } else if (param_name == name_ + "." + "corridor_cost") {
        const int new_value = parameter.as_int();
        if (new_value <= 0 || new_value > 254) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %d, "
            "it should be between 1 and 254. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason = "corridor_cost must be between 1 and 254";
        }
      } else if (param_name == name_ + "." + "wall_thickness") {
        const int new_value = parameter.as_int();
        if (new_value <= 0) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %d, "
            "it should be > 0. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason = "wall_thickness must be greater than zero";
        }
      } else if (param_name == name_ + "." + "cost_write_mode") {
        const int new_value = parameter.as_int();
        if (new_value < 0 || new_value > 2) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %d, "
            "it should be 0, 1, or 2. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason =
            "cost_write_mode must be 0 (corridor), 1 (fill max), or 2 (fill overwrite)";
        }
      }
    }
  }

  return result;
}

void
BoundedTrackingErrorLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> guard(data_mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      const bool is_look_ahead = param_name == name_ + "." + "look_ahead";
      const bool is_corridor_width = param_name == name_ + "." + "corridor_width";
      if (is_look_ahead && look_ahead_ != parameter.as_double()) {
        look_ahead_ = parameter.as_double();
        current_ = false;
      } else if (is_corridor_width && corridor_width_ != parameter.as_double()) {
        corridor_width_ = parameter.as_double();
        current_ = false;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      const bool is_enabled = param_name == name_ + "." + "enabled";
      if (is_enabled && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      const bool is_step = param_name == name_ + "." + "step";
      const bool is_cost = param_name == name_ + "." + "corridor_cost";
      const bool is_thickness = param_name == name_ + "." + "wall_thickness";
      const bool is_write_mode = param_name == name_ + "." + "cost_write_mode";
      if (is_step) {
        const int new_step = parameter.as_int();
        if (static_cast<size_t>(new_step) != step_size_) {
          step_size_ = static_cast<size_t>(new_step);
          current_ = false;
        }
      } else if (is_cost && corridor_cost_ != static_cast<unsigned char>(parameter.as_int())) {
        corridor_cost_ = static_cast<unsigned char>(parameter.as_int());
        current_ = false;
      } else if (is_thickness && wall_thickness_ != parameter.as_int()) {
        wall_thickness_ = parameter.as_int();
        current_ = false;
      } else if (is_write_mode && cost_write_mode_ != parameter.as_int()) {
        cost_write_mode_ = parameter.as_int();
        current_ = false;
      }
    }
  }
}

}  // namespace nav2_costmap_2d
