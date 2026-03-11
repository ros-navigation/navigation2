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
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BoundedTrackingErrorLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

BoundedTrackingErrorLayer::~BoundedTrackingErrorLayer()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  if (on_set_params_handler_) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  if (post_set_params_handler_) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
}

void BoundedTrackingErrorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  tracking_feedback_topic_ = node->declare_or_get_parameter(
    name_ + "." + "tracking_feedback_topic", std::string("tracking_feedback"));

  path_topic_ = node->declare_or_get_parameter(
    name_ + "." + "path_topic", std::string("plan"));

  look_ahead_ = node->declare_or_get_parameter(name_ + "." + "look_ahead", 2.5);

  step_ = node->declare_or_get_parameter(name_ + "." + "step", 10);
  step_size_ = static_cast<size_t>(step_);

  corridor_width_ = node->declare_or_get_parameter(name_ + "." + "corridor_width", 2.0);

  wall_thickness_ = node->declare_or_get_parameter(name_ + "." + "wall_thickness", 1);

  path_segment_resolution_ = node->declare_or_get_parameter(
    name_ + "." + "path_segment_resolution", 5);

  int corridor_cost_param = node->declare_or_get_parameter(name_ + "." + "corridor_cost", 190);
  corridor_cost_ = static_cast<unsigned char>(std::clamp(corridor_cost_param, 1, 254));

  bool enabled_param = node->declare_or_get_parameter(name_ + "." + "enabled", true);
  enabled_.store(enabled_param);

  double temp_tf_tol = 0.1;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  if (wall_thickness_ <= 0) {
    throw std::runtime_error{"wall_thickness must be greater than zero"};
  }
  if (look_ahead_ <= 0.0 || look_ahead_ > 3.0) {
    throw std::runtime_error{"look_ahead must be positive and <= 3.0 meters"};
  }
  if (corridor_width_ <= 0.0) {
    throw std::runtime_error{"corridor_width must be positive"};
  }
  if (step_size_ == 0) {
    throw std::runtime_error{"step must be greater than zero"};
  }
  if (path_segment_resolution_ < 1) {
    throw std::runtime_error{"path_segment_resolution must be at least 1"};
  }

  // Join topics with parent namespace for proper namespacing
  std::string path_topic = joinWithParentNamespace(path_topic_);
  std::string tracking_feedback_topic = joinWithParentNamespace(tracking_feedback_topic_);

  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic,
    std::bind(&BoundedTrackingErrorLayer::pathCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS()
  );

  tracking_feedback_sub_ = node->create_subscription<nav2_msgs::msg::TrackingFeedback>(
    tracking_feedback_topic,
    std::bind(&BoundedTrackingErrorLayer::trackingCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS()
  );

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    std::bind(&BoundedTrackingErrorLayer::goalCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS()
  );
}

void BoundedTrackingErrorLayer::activate()
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

  enabled_.store(true);
  current_ = true;
}

void BoundedTrackingErrorLayer::deactivate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (on_set_params_handler_) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
    on_set_params_handler_.reset();
  }
  if (post_set_params_handler_) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
    post_set_params_handler_.reset();
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    resetState();
  }

  enabled_.store(false);
}

void BoundedTrackingErrorLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  resetState();
  current_ = true;
}

void BoundedTrackingErrorLayer::resetState()
{
  last_path_ = nav_msgs::msg::Path();
  current_path_index_.store(0);
  last_goal_ = geometry_msgs::msg::PoseStamped();
}

void BoundedTrackingErrorLayer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if this is a new goal (different position from last goal)
  bool is_new_goal = false;
  if (last_goal_.header.frame_id.empty()) {
    is_new_goal = true;
  } else {
    const double dx = msg->pose.position.x - last_goal_.pose.position.x;
    const double dy = msg->pose.position.y - last_goal_.pose.position.y;
    const double distance = std::hypot(dx, dy);

    if (distance > 0.1) {
      is_new_goal = true;
    }
  }

  if (is_new_goal) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "New goal received, clearing corridor");
    resetState();
  }

  last_goal_ = *msg;
}

void BoundedTrackingErrorLayer::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto now = node->now();
  const auto msg_time = rclcpp::Time(msg->header.stamp);
  const auto age = (now - msg_time).seconds();

  if (age > 2.0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Path is %.2f seconds old, clearing corridor", age);
    last_path_ = nav_msgs::msg::Path();
    return;
  }

  // Check if path was updated (same goal, different path)
  nav_msgs::msg::Path new_path = *msg;
  if (nav2_util::isPathUpdated(new_path, last_path_)) {
    RCLCPP_DEBUG(node->get_logger(), "Path updated, resetting state");
    resetState();
  }

  last_path_ = *msg;
}

void BoundedTrackingErrorLayer::trackingCallback(
  const nav2_msgs::msg::TrackingFeedback::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  const auto now = node->now();
  const auto msg_time = rclcpp::Time(msg->header.stamp);
  const auto age = (now - msg_time).seconds();

  if (age > 1.0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Tracking feedback is %.2f seconds old", age);
    return;
  }

  current_path_index_.store(msg->current_path_index);
}

void BoundedTrackingErrorLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{

  if (!enabled_.load()) {
    return;
  }

  *min_x = std::min(*min_x, robot_x - look_ahead_);
  *max_x = std::max(*max_x, robot_x + look_ahead_);
  *min_y = std::min(*min_y, robot_y - look_ahead_);
  *max_y = std::max(*max_y, robot_y + look_ahead_);
}

void BoundedTrackingErrorLayer::getWallPolygons(
  const nav_msgs::msg::Path & segment,
  WallPolygons & walls)
{
  if (segment.poses.empty() || step_size_ == 0) {
    walls.clearAndReserve(0);
    return;
  }

  const double resolution = layered_costmap_->getCostmap()->getResolution();
  const double wall_outer_offset = (corridor_width_ * 0.5) + (wall_thickness_ * resolution);

  const size_t estimated_points = (segment.poses.size() / step_size_) + 1;
  walls.clearAndReserve(estimated_points);

  for (size_t current_index = 0; current_index < segment.poses.size();
    current_index += step_size_)
  {
    const auto & current_pose = segment.poses[current_index];
    const double px = current_pose.pose.position.x;
    const double py = current_pose.pose.position.y;

    if (current_index + step_size_ >= segment.poses.size()) {
      break;
    }

    const auto & next_pose = segment.poses[current_index + step_size_];
    const double dx = next_pose.pose.position.x - px;
    const double dy = next_pose.pose.position.y - py;

    const double norm = std::hypot(dx, dy);
    if (norm < resolution) {
      continue;
    }

    const double inv_norm = 1.0 / norm;
    const double perp_x = -dy * inv_norm;
    const double perp_y = dx * inv_norm;
    const double tang_x = dx * inv_norm;
    const double tang_y = dy * inv_norm;

    walls.left_outer.push_back(
      {px + perp_x * wall_outer_offset, py + perp_y * wall_outer_offset});
    walls.right_outer.push_back(
      {px - perp_x * wall_outer_offset, py - perp_y * wall_outer_offset});
    walls.perpendiculars.push_back({perp_x, perp_y});
    walls.tangents.push_back({tang_x, tang_y});
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
  const size_t jump = static_cast<size_t>(path_segment_resolution_);

  for (size_t i = start_index; i < path.poses.size() - 1; i += jump) {
    const size_t next_i = std::min(i + jump, path.poses.size() - 1);

    // Calculate straight-line distance between jumped poses
    dist_traversed += nav2_util::geometry_utils::euclidean_distance(
      path.poses[i], path.poses[next_i]);

    end_index = next_i;
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


void BoundedTrackingErrorLayer::fillThicknessPixels(
  unsigned char * costmap,
  unsigned int size_x,
  unsigned int size_y,
  unsigned int curr_x,
  unsigned int curr_y,
  double perp_x,
  double perp_y,
  int perp_sign)
{
  // Pre-compute the perpendicular offset as integer pixel steps
  for (int offset = 1; offset < wall_thickness_; ++offset) {
    // Calculate pixel offset directly using perpendicular direction
    // perp_sign determines whether we go inward (+) or outward (-)
    const int ox = static_cast<int>(curr_x) -
      static_cast<int>(std::round(perp_sign * perp_x * offset));
    const int oy = static_cast<int>(curr_y) -
      static_cast<int>(std::round(perp_sign * perp_y * offset));

    if (ox >= 0 && ox < static_cast<int>(size_x) &&
      oy >= 0 && oy < static_cast<int>(size_y))
    {
      costmap[oy * size_x + ox] = corridor_cost_;
    }
  }
}

void BoundedTrackingErrorLayer::fillDiagonalGaps(
  unsigned char * costmap,
  unsigned int size_x,
  unsigned int size_y,
  unsigned int curr_x,
  unsigned int curr_y,
  double tang_x,
  double tang_y)
{
  // Determine gap fill direction based on tangent
  const int gap_dx = (tang_x > 0) ? 1 : -1;
  const int gap_dy = (tang_y > 0) ? 1 : -1;

  // Fill two adjacent pixels to close diagonal gap
  const int gap_x1 = static_cast<int>(curr_x) + gap_dx;
  const int gap_y1 = static_cast<int>(curr_y);
  const int gap_x2 = static_cast<int>(curr_x);
  const int gap_y2 = static_cast<int>(curr_y) + gap_dy;

  if (gap_x1 >= 0 && gap_x1 < static_cast<int>(size_x) &&
    gap_y1 >= 0 && gap_y1 < static_cast<int>(size_y))
  {
    costmap[gap_y1 * size_x + gap_x1] = corridor_cost_;
  }
  if (gap_x2 >= 0 && gap_x2 < static_cast<int>(size_x) &&
    gap_y2 >= 0 && gap_y2 < static_cast<int>(size_y))
  {
    costmap[gap_y2 * size_x + gap_x2] = corridor_cost_;
  }
}

void BoundedTrackingErrorLayer::drawWallSegment(
  unsigned char * costmap,
  unsigned int size_x,
  unsigned int size_y,
  const nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<std::array<double, 2>> & wall_points,
  const std::vector<std::array<double, 2>> & perpendiculars,
  const std::vector<std::array<double, 2>> & tangents,
  int perp_sign)
{
  if (wall_points.size() < 2) {
    return;
  }

  for (size_t i = 0; i < wall_points.size() - 1; ++i) {
    unsigned int x0, y0, x1, y1;

    if (!master_grid.worldToMap(wall_points[i][0], wall_points[i][1], x0, y0)) {
      continue;
    }
    if (!master_grid.worldToMap(wall_points[i + 1][0], wall_points[i + 1][1], x1, y1)) {
      continue;
    }

    const double tang_x = tangents[i][0];
    const double tang_y = tangents[i][1];

    const bool is_diagonal = (std::abs(tang_x) > 0.1 && std::abs(tang_y) > 0.1);

    nav2_util::LineIterator line(x0, y0, x1, y1);

    for (; line.isValid(); line.advance()) {
      const unsigned int curr_x = line.getX();
      const unsigned int curr_y = line.getY();

      const unsigned int index = curr_y * size_x + curr_x;
      costmap[index] = corridor_cost_;

      fillThicknessPixels(
        costmap, size_x, size_y,
        curr_x, curr_y,
        perpendiculars[i][0], perpendiculars[i][1],
        perp_sign);

      if (is_diagonal) {
        fillDiagonalGaps(
          costmap, size_x, size_y,
          curr_x, curr_y,
          tang_x, tang_y);
      }
    }
  }
}

void BoundedTrackingErrorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_.load()) {
    return;
  }

  nav_msgs::msg::Path cached_path;
  size_t cached_path_index;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    cached_path = last_path_;
    cached_path_index = current_path_index_.load();
  }

  getPathSegment(cached_path, cached_path_index, segment_buffer_);

  if (segment_buffer_.poses.size() < 10) {
    RCLCPP_INFO_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Path segment too small (%zu poses), skipping wall generation",
      segment_buffer_.poses.size());
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();

  transformed_segment_buffer_.poses.clear();
  if (!nav2_util::transformPathInTargetFrame(
      segment_buffer_, transformed_segment_buffer_, *tf_, costmap_frame,
      tf2::durationToSec(transform_tolerance_)))
  {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Failed to transform path to %s, skipping wall generation",
      costmap_frame.c_str());
    return;
  }

  getWallPolygons(transformed_segment_buffer_, walls_buffer_);

  if (walls_buffer_.isEmpty()) {
    RCLCPP_INFO_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Not enough wall points to form polygons");
    return;
  }

  // Cache grid properties to avoid repeated function calls
  unsigned char * costmap = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  // Draw left wall (perp_sign = -1: thickness goes inward, subtracting perpendicular)
  drawWallSegment(
    costmap, size_x, size_y,
    master_grid,
    walls_buffer_.left_outer,
    walls_buffer_.perpendiculars,
    walls_buffer_.tangents,
    -1);

  // Draw right wall (perp_sign = +1: thickness goes inward, adding perpendicular)
  drawWallSegment(
    costmap, size_x, size_y,
    master_grid,
    walls_buffer_.right_outer,
    walls_buffer_.perpendiculars,
    walls_buffer_.tangents,
    +1);
}

rcl_interfaces::msg::SetParametersResult BoundedTrackingErrorLayer::validateParameterUpdatesCallback
(
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
        if (new_value < 0.0) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %f, "
            "it should be >= 0. Rejecting parameter update.",
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
      } else if (param_name == name_ + "." + "path_segment_resolution") {
        const int new_value = parameter.as_int();
        if (new_value < 1) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is incorrectly set to %d, "
            "it should be >= 1. Rejecting parameter update.",
            param_name.c_str(), new_value);
          result.successful = false;
          result.reason = "path_segment_resolution must be at least 1";
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
        enabled_.store(parameter.as_bool());
        current_ = false;
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "step" &&
        step_ != parameter.as_int())
      {
        step_ = parameter.as_int();
        step_size_ = static_cast<size_t>(step_);
        current_ = false;
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
      } else if (param_name == name_ + "." + "path_segment_resolution" &&
        path_segment_resolution_ != parameter.as_int())
      {
        path_segment_resolution_ = parameter.as_int();
        current_ = false;
      }
    }
  }
}

}  // namespace nav2_costmap_2d
