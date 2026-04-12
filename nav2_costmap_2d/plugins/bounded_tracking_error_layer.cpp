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

#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>

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
    nav2::qos::StandardTopicQoS()
  );

  current_ = true;
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
BoundedTrackingErrorLayer::resetState()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_path_ptr_.reset();
  current_path_index_.store(0);
}

void
BoundedTrackingErrorLayer::matchSize()
{
  resolution_ = layered_costmap_->getCostmap()->getResolution();
  costmap_frame_ = layered_costmap_->getGlobalFrameID();
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

  fill_outside_corridor_ = node->declare_or_get_parameter(
    name_ + "." + "fill_outside_corridor", false);

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);

  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  node->get_parameter("robot_base_frame", robot_base_frame_);
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
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  if (resolution_ <= 0.0) {
    return;
  }

  nav_msgs::msg::Path::ConstSharedPtr path_ptr;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    path_ptr = last_path_ptr_;
  }

  if (!path_ptr || path_ptr->poses.empty()) {
    return;
  }

  const auto age = (clock_->now() - rclcpp::Time(path_ptr->header.stamp)).seconds();
  if (age > 5.0) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Path is %.2f seconds old, clearing corridor state", age);
    resetState();
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
      5000,
      "Failed to get robot pose in %s, skipping corridor update",
      costmap_frame_.c_str());
    return;
  }

  size_t path_index = current_path_index_.load();
  if (path_index >= path_ptr->poses.size()) {
    path_index = 0;
    current_path_index_.store(0);
  }

  const auto search_result = nav2_util::distance_from_path(
    *path_ptr,
    robot_pose.pose,
    path_index,
    look_ahead_);
  current_path_index_.store(search_result.closest_segment_index);

  // For wall-only mode use look-ahead segment; for fill mode use full path.
  const nav_msgs::msg::Path & active_path = fill_outside_corridor_ ?
    *path_ptr :
    (getPathSegment(*path_ptr, search_result.closest_segment_index, segment_buffer_),
    segment_buffer_);

  if (!fill_outside_corridor_) {
    const size_t min_poses = (step_size_ * 2) + 1;
    if (segment_buffer_.poses.size() < min_poses) {
      RCLCPP_DEBUG_THROTTLE(
        logger_,
        *clock_,
        5000,
        "Path segment too small (%zu poses), need at least %zu poses for step_size=%zu",
        segment_buffer_.poses.size(), min_poses, step_size_);
      return;
    }
  }

  const nav_msgs::msg::Path * transformed_ptr = nullptr;

  if (active_path.header.frame_id == costmap_frame_) {
    transformed_ptr = &active_path;
  } else {
    if (!nav2_util::transformPathInTargetFrame(
        active_path, transformed_segment_buffer_, *tf_, costmap_frame_, tf_tol))
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *clock_,
        5000,
        "Failed to transform path to %s, skipping wall generation",
        costmap_frame_.c_str());
      return;
    }
    transformed_ptr = &transformed_segment_buffer_;
  }

  getWallPolygons(*transformed_ptr, walls_buffer_);

  if (fill_outside_corridor_) {
    saveCorridorInterior(master_grid, walls_buffer_);
    fillOutsideCorridor(master_grid, min_i, min_j, max_i, max_j);
  }

  drawCorridorWalls(master_grid, walls_buffer_.left_inner, walls_buffer_.left_outer);
  drawCorridorWalls(master_grid, walls_buffer_.right_inner, walls_buffer_.right_outer);
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

  if (path_index < end_index && end_index <= path.poses.size()) {
    segment.header = path.header;
    segment.poses.assign(
      path.poses.begin() + path_index,
      path.poses.begin() + end_index + 1
    );
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

  for (size_t current_index = 0; current_index < segment.poses.size();
    current_index += step_size_)
  {
    const auto & current_pose = segment.poses[current_index];
    const double px = current_pose.pose.position.x;
    const double py = current_pose.pose.position.y;

    size_t next_index = current_index + step_size_;
    if (next_index >= segment.poses.size()) {
      next_index = segment.poses.size() - 1;
      if (next_index == current_index) {
        break;
      }
    }

    const auto & next_pose = segment.poses[next_index];
    const double dx = next_pose.pose.position.x - px;
    const double dy = next_pose.pose.position.y - py;

    const double norm = std::hypot(dx, dy);
    if (norm < resolution_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Skipping degenerate path segment at index %zu: norm %.4f < resolution %.4f",
        current_index, norm, resolution_);
      continue;
    }

    // Unit vector perpendicular to the path direction (rotate 90 CCW).
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
  }
}

void
BoundedTrackingErrorLayer::drawCorridorWalls(
  nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<std::array<double, 2>> & inner_points,
  const std::vector<std::array<double, 2>> & outer_points)
{
  if (inner_points.size() < 2 || outer_points.size() < 2) {
    RCLCPP_DEBUG(logger_, "Skipping corridor wall draw: insufficient boundary points");
    return;
  }

  const size_t num_segments = std::min(inner_points.size(), outer_points.size()) - 1;

  for (size_t i = 0; i < num_segments; ++i) {
    CellPoint inner0, inner1, outer0, outer1;

    if (!master_grid.worldToMap(inner_points[i][0], inner_points[i][1], inner0.x, inner0.y)) {
      continue;
    }
    if (!master_grid.worldToMap(
        inner_points[i + 1][0], inner_points[i + 1][1], inner1.x, inner1.y))
    {
      continue;
    }
    if (!master_grid.worldToMap(outer_points[i][0], outer_points[i][1], outer0.x, outer0.y)) {
      continue;
    }
    if (!master_grid.worldToMap(
        outer_points[i + 1][0], outer_points[i + 1][1], outer1.x, outer1.y))
    {
      continue;
    }

    fillCorridorQuad(master_grid, inner0, inner1, outer0, outer1);
  }
}

void
BoundedTrackingErrorLayer::traceEdge(
  CellPoint p0, CellPoint p1, int clamped_y_min, int height)
{
  nav2_util::LineIterator line(p0.x, p0.y, p1.x, p1.y);
  for (; line.isValid(); line.advance()) {
    const int x = static_cast<int>(line.getX());
    const int y = static_cast<int>(line.getY());
    const int buffer_idx = y - clamped_y_min;

    if (buffer_idx >= 0 && buffer_idx < height) {
      span_x_min_buffer_[buffer_idx] = std::min(span_x_min_buffer_[buffer_idx], x);
      span_x_max_buffer_[buffer_idx] = std::max(span_x_max_buffer_[buffer_idx], x);
    }
  }
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

  if (clamped_y_min != y_min || clamped_y_max != y_max) {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 1000,
      "Quad partially outside costmap bounds, clipping Y from [%d, %d] to [%d, %d]",
      y_min, y_max, clamped_y_min, clamped_y_max);
  }

  if (clamped_y_min > clamped_y_max) {
    return;
  }

  const int height = clamped_y_max - clamped_y_min + 1;

  span_x_min_buffer_.assign(height, std::numeric_limits<int>::max());
  span_x_max_buffer_.assign(height, std::numeric_limits<int>::min());

  traceEdge(inner0, inner1, clamped_y_min, height);  // Inner edge
  traceEdge(inner1, outer1, clamped_y_min, height);  // Right edge
  traceEdge(outer1, outer0, clamped_y_min, height);  // Outer edge
  traceEdge(outer0, inner0, clamped_y_min, height);  // Left edge

  for (int buffer_idx = 0; buffer_idx < height; ++buffer_idx) {
    const int y = clamped_y_min + buffer_idx;
    const int x_min = span_x_min_buffer_[buffer_idx];
    const int x_max = span_x_max_buffer_[buffer_idx];

    if (x_min > x_max) {
      continue;
    }

    const int x_start = std::max(x_min, 0);
    const int x_end = std::min(x_max, static_cast<int>(size_x) - 1);

    // Preserve any higher cost already written by an earlier plugin this cycle.
    for (int x = x_start; x <= x_end; ++x) {
      costmap[y * size_x + x] = std::max(costmap[y * size_x + x], corridor_cost_);
    }
  }
}

void
BoundedTrackingErrorLayer::saveCorridorInterior(
  nav2_costmap_2d::Costmap2D & master_grid,
  const WallPolygons & walls)
{
  corridor_index_set_.clear();

  if (walls.left_inner.size() < 2 || walls.right_inner.size() < 2) {
    return;
  }

  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  const size_t num_segments = std::min(walls.left_inner.size(), walls.right_inner.size()) - 1;

  for (size_t i = 0; i < num_segments; ++i) {
    CellPoint left0, left1, right0, right1;

    if (!master_grid.worldToMap(
        walls.left_inner[i][0], walls.left_inner[i][1], left0.x, left0.y)) {continue;}
    if (!master_grid.worldToMap(
        walls.left_inner[i + 1][0], walls.left_inner[i + 1][1], left1.x, left1.y)) {continue;}
    if (!master_grid.worldToMap(
        walls.right_inner[i][0], walls.right_inner[i][1], right0.x, right0.y)) {continue;}
    if (!master_grid.worldToMap(
        walls.right_inner[i + 1][0], walls.right_inner[i + 1][1], right1.x, right1.y)) {continue;}

    const int y_min = std::min({left0.y, left1.y, right0.y, right1.y});
    const int y_max = std::max({left0.y, left1.y, right0.y, right1.y});
    const int clamped_y_min = std::max(y_min, 0);
    const int clamped_y_max = std::min(y_max, static_cast<int>(size_y) - 1);

    if (clamped_y_min > clamped_y_max) {continue;}

    const int height = clamped_y_max - clamped_y_min + 1;
    span_x_min_buffer_.assign(height, std::numeric_limits<int>::max());
    span_x_max_buffer_.assign(height, std::numeric_limits<int>::min());

    traceEdge(left0, left1, clamped_y_min, height);    // Left inner edge
    traceEdge(left1, right1, clamped_y_min, height);   // Far cap
    traceEdge(right1, right0, clamped_y_min, height);  // Right inner edge
    traceEdge(right0, left0, clamped_y_min, height);   // Near cap

    for (int buffer_idx = 0; buffer_idx < height; ++buffer_idx) {
      const int y = clamped_y_min + buffer_idx;
      const int x_min = span_x_min_buffer_[buffer_idx];
      const int x_max = span_x_max_buffer_[buffer_idx];

      if (x_min > x_max) {continue;}

      const int x_start = std::max(x_min, 0);
      const int x_end = std::min(x_max, static_cast<int>(size_x) - 1);

      for (int x = x_start; x <= x_end; ++x) {
        corridor_index_set_.insert(static_cast<unsigned int>(y) * size_x +
          static_cast<unsigned int>(x));
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
      if (corridor_index_set_.count(flat_idx)) {
        continue;
      }
      costmap[flat_idx] = std::max(costmap[flat_idx], corridor_cost_);
    }
  }
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
      if (param_name == name_ + "." + "look_ahead" &&
        look_ahead_ != parameter.as_double())
      {
        look_ahead_ = parameter.as_double();
        current_ = false;
      } else if (param_name == name_ + "." + "corridor_width" &&
        corridor_width_ != parameter.as_double())
      {
        corridor_width_ = parameter.as_double();
        current_ = false;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      } else if (param_name == name_ + "." + "fill_outside_corridor" &&
        fill_outside_corridor_ != parameter.as_bool())
      {
        fill_outside_corridor_ = parameter.as_bool();
        current_ = false;
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "step") {
        const int new_step = parameter.as_int();
        if (static_cast<size_t>(new_step) != step_size_) {
          step_size_ = static_cast<size_t>(new_step);
          current_ = false;
        }
      } else if (param_name == name_ + "." + "corridor_cost" &&
        corridor_cost_ != static_cast<unsigned char>(parameter.as_int()))
      {
        corridor_cost_ = static_cast<unsigned char>(parameter.as_int());
        current_ = false;
      } else if (param_name == name_ + "." + "wall_thickness" &&
        wall_thickness_ != parameter.as_int())
      {
        wall_thickness_ = parameter.as_int();
        current_ = false;
      }
    }
  }
}

}  // namespace nav2_costmap_2d
