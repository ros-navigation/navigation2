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
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/execution_timer.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BoundedTrackingErrorLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

void BoundedTrackingErrorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  path_topic_ = node->declare_or_get_parameter(
    name_ + "." + "path_topic", std::string("plan"));

  look_ahead_ = node->declare_or_get_parameter(name_ + "." + "look_ahead", 2.5);

  const int step_param = node->declare_or_get_parameter(name_ + "." + "step", 10);
  if (step_param <= 0) {
    throw std::runtime_error{"step must be greater than zero"};
  }
  step_size_ = static_cast<size_t>(step_param);

  corridor_width_ = node->declare_or_get_parameter(name_ + "." + "corridor_width", 2.0);

  wall_thickness_ = node->declare_or_get_parameter(name_ + "." + "wall_thickness", 1);

  int corridor_cost_param = node->declare_or_get_parameter(name_ + "." + "corridor_cost", 190);
  corridor_cost_ = static_cast<unsigned char>(std::clamp(corridor_cost_param, 1, 254));

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);

  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  if (wall_thickness_ <= 0) {
    throw std::runtime_error{"wall_thickness must be greater than zero"};
  }
  if (look_ahead_ <= 0.0) {
    throw std::runtime_error{"look_ahead must be positive"};
  }
  if (corridor_width_ <= 0.0) {
    throw std::runtime_error{"corridor_width must be positive"};
  }


  // Join topics with parent namespace for proper namespacing
  std::string path_topic = joinWithParentNamespace(path_topic_);

  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic,
    std::bind(&BoundedTrackingErrorLayer::pathCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS()
  );
}

void BoundedTrackingErrorLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  costmap_frame_ = layered_costmap_->getGlobalFrameID();
}

void BoundedTrackingErrorLayer::activate()
{
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &BoundedTrackingErrorLayer::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &BoundedTrackingErrorLayer::validateParameterUpdatesCallback,
      this, std::placeholders::_1));

  current_ = true;
}

void BoundedTrackingErrorLayer::deactivate()
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

void BoundedTrackingErrorLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  resetState();
  current_ = false;
}

void BoundedTrackingErrorLayer::resetState()
{
  last_path_ptr_.reset();
  current_path_index_.store(0);
}

void BoundedTrackingErrorLayer::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto now = clock_->now();
  const auto msg_time = rclcpp::Time(msg->header.stamp);
  const auto age = (now - msg_time).seconds();

  // Discard paths older than 2 seconds to avoid stale corridor generation
  if (age > 2.0) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Path is %.2f seconds old, clearing corridor", age);
    last_path_ptr_.reset();
    return;
  }

  // Check if path was updated
  if (last_path_ptr_) {
    nav_msgs::msg::Path new_path = *msg;
    nav_msgs::msg::Path old_path = *last_path_ptr_;
    if (nav2_util::isPathUpdated(new_path, old_path)) {
      RCLCPP_DEBUG(logger_, "Path updated, resetting state");
      resetState();
    }
  }

  last_path_ptr_ = msg;
}



void BoundedTrackingErrorLayer::updateBounds(
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

void BoundedTrackingErrorLayer::getWallPolygons(
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
      continue;
    }

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

void BoundedTrackingErrorLayer::getPathSegment(
  const nav_msgs::msg::Path & path,
  size_t path_index,
  nav_msgs::msg::Path & segment)
{
  segment.poses.clear();

  if (path.poses.empty() || path_index >= path.poses.size()) {
    return;
  }

  const size_t start_index = path_index;
  size_t end_index = start_index;
  double dist_traversed = 0.0;

  for (size_t i = start_index; i < path.poses.size() - 1; i++) {
    dist_traversed += nav2_util::geometry_utils::euclidean_distance(
      path.poses[i], path.poses[i + 1]);

    end_index = i + 1;
    if (dist_traversed >= look_ahead_) {
      break;
    }
  }

  if (start_index < end_index && end_index <= path.poses.size()) {
    segment.header = path.header;
    segment.poses.assign(
      path.poses.begin() + start_index,
      path.poses.begin() + end_index
    );
  } else if (start_index == path.poses.size() - 1) {
    segment.header = path.header;
    segment.poses.push_back(path.poses[start_index]);
  }
}

// Fills a convex quadrilateral using scanline span-buffer rasterization.
// Each edge is rasterized with Bresenham's line algorithm to build per-row [x_min, x_max]
// spans, which are then filled horizontally.
void BoundedTrackingErrorLayer::fillCorridorQuad(
  unsigned char * costmap,
  unsigned int size_x,
  unsigned int size_y,
  unsigned int inner_x0,
  unsigned int inner_y0,
  unsigned int inner_x1,
  unsigned int inner_y1,
  unsigned int outer_x0,
  unsigned int outer_y0,
  unsigned int outer_x1,
  unsigned int outer_y1)
{
  // Quad vertices: inner[i], inner[i+1], outer[i+1], outer[i]
  // Find bounding box
  const int y_min = std::min({inner_y0, inner_y1, outer_y0, outer_y1});
  const int y_max = std::max({inner_y0, inner_y1, outer_y0, outer_y1});

  // Clamp Y bounds to costmap instead of discarding partially-visible quads
  const int clamped_y_min = std::max(y_min, 0);
  const int clamped_y_max = std::min(y_max, static_cast<int>(size_y) - 1);

  if (clamped_y_min > clamped_y_max) {
    return;
  }

  // Span buffer: for each y, track [x_min, x_max]
  const int height = clamped_y_max - clamped_y_min + 1;

  if (height <= 0 || height > static_cast<int>(size_y)) {
    RCLCPP_WARN(
      logger_,
      "Invalid quad height: %d (size_y: %u). Skipping quad.",
      height, size_y);
    return;
  }

  span_x_min_buffer_.assign(height, std::numeric_limits<int>::max());
  span_x_max_buffer_.assign(height, std::numeric_limits<int>::min());

  auto trace_edge = [&](unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
      nav2_util::LineIterator line(x0, y0, x1, y1);
      for (; line.isValid(); line.advance()) {
        const int x = static_cast<int>(line.getX());
        const int y = static_cast<int>(line.getY());
        const int buffer_idx = y - clamped_y_min;

        if (buffer_idx >= 0 && buffer_idx < height) {
          span_x_min_buffer_[buffer_idx] = std::min(span_x_min_buffer_[buffer_idx], x);
          span_x_max_buffer_[buffer_idx] = std::max(span_x_max_buffer_[buffer_idx], x);
        }
      }
    };

  // Trace all 4 edges of the quad
  trace_edge(inner_x0, inner_y0, inner_x1, inner_y1);  // Inner edge
  trace_edge(inner_x1, inner_y1, outer_x1, outer_y1);  // Right edge
  trace_edge(outer_x1, outer_y1, outer_x0, outer_y0);  // Outer edge
  trace_edge(outer_x0, outer_y0, inner_x0, inner_y0);  // Left edge

  for (int buffer_idx = 0; buffer_idx < height; ++buffer_idx) {
    const int y = clamped_y_min + buffer_idx;
    const int x_min = span_x_min_buffer_[buffer_idx];
    const int x_max = span_x_max_buffer_[buffer_idx];

    if (x_min > x_max) {
      continue;
    }

    // Clamp to costmap bounds
    const int x_start = std::max(x_min, 0);
    const int x_end = std::min(x_max, static_cast<int>(size_x) - 1);

    // Fill the horizontal span
    for (int x = x_start; x <= x_end; ++x) {
      costmap[y * size_x + x] = corridor_cost_;
    }
  }
}

void BoundedTrackingErrorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  nav2_util::ExecutionTimer timer;
  timer.start();

  if (!enabled_) {
    return;
  }

  if (resolution_ <= 0.0) {
    return;
  }

  nav_msgs::msg::Path::ConstSharedPtr cached_path_ptr;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    cached_path_ptr = last_path_ptr_;
  }

  if (!cached_path_ptr) {
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, costmap_frame_, "base_link",
      tf2::durationToSec(transform_tolerance_)))
  {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Failed to get robot pose in %s, skipping corridor update",
      costmap_frame_.c_str());
    return;
  }

  const size_t cached_path_index = current_path_index_.load();
  const auto search_result = nav2_util::distance_from_path(
    *cached_path_ptr,
    robot_pose.pose,
    cached_path_index,
    look_ahead_);
  current_path_index_.store(search_result.closest_segment_index);

  getPathSegment(*cached_path_ptr, search_result.closest_segment_index, segment_buffer_);

  const size_t min_poses = (step_size_ * 2) + 1;
  if (segment_buffer_.poses.size() < min_poses) {
    RCLCPP_INFO_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Path segment too small (%zu poses), need at least %zu poses for step_size=%zu",
      segment_buffer_.poses.size(), min_poses, step_size_);
    return;
  }

  if (segment_buffer_.header.frame_id == costmap_frame_) {
    transformed_segment_buffer_ = segment_buffer_;
  } else {
    if (!nav2_util::transformPathInTargetFrame(
        segment_buffer_, transformed_segment_buffer_, *tf_, costmap_frame_,
        tf2::durationToSec(transform_tolerance_)))
    {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *clock_,
        5000,
        "Failed to transform path to %s, skipping wall generation",
        costmap_frame_.c_str());
      return;
    }
  }

  getWallPolygons(transformed_segment_buffer_, walls_buffer_);



  // Cache grid properties to avoid repeated function calls
  unsigned char * costmap = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  drawCorridorWalls(
    costmap, size_x, size_y,
    master_grid,
    walls_buffer_.left_inner,
    walls_buffer_.left_outer);

  drawCorridorWalls(
    costmap, size_x, size_y,
    master_grid,
    walls_buffer_.right_inner,
    walls_buffer_.right_outer);

  timer.end();
  RCLCPP_INFO(
    logger_,
    "BoundedTrackingErrorLayer::updateCosts execution time: %.6f seconds",
    timer.elapsed_time_in_seconds());
}

void BoundedTrackingErrorLayer::drawCorridorWalls(
  unsigned char * costmap,
  unsigned int size_x,
  unsigned int size_y,
  const nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<std::array<double, 2>> & inner_points,
  const std::vector<std::array<double, 2>> & outer_points)
{
  if (inner_points.size() < 2 || outer_points.size() < 2) {
    return;
  }

  // Process each segment as a quad
  const size_t num_segments = std::min(inner_points.size(), outer_points.size()) - 1;

  for (size_t i = 0; i < num_segments; ++i) {
    unsigned int inner_x0, inner_y0, inner_x1, inner_y1;
    unsigned int outer_x0, outer_y0, outer_x1, outer_y1;

    if (!master_grid.worldToMap(inner_points[i][0], inner_points[i][1], inner_x0, inner_y0)) {
      continue;
    }
    if (!master_grid.worldToMap(
        inner_points[i + 1][0], inner_points[i + 1][1], inner_x1,
        inner_y1))
    {
      continue;
    }
    if (!master_grid.worldToMap(outer_points[i][0], outer_points[i][1], outer_x0, outer_y0)) {
      continue;
    }
    if (!master_grid.worldToMap(
        outer_points[i + 1][0], outer_points[i + 1][1], outer_x1,
        outer_y1))
    {
      continue;
    }

    fillCorridorQuad(
      costmap, size_x, size_y,
      inner_x0, inner_y0, inner_x1, inner_y1,
      outer_x0, outer_y0, outer_x1, outer_y1);
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

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
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
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
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

void BoundedTrackingErrorLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> guard(data_mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
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
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
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
