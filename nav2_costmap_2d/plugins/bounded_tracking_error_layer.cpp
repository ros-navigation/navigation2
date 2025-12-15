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
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void BoundedTrackingErrorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("tracking_feedback_topic", rclcpp::ParameterValue("tracking_feedback"));
  node->get_parameter(name_ + "." + "tracking_feedback_topic", tracking_feedback_topic_);

  declareParameter("path_topic", rclcpp::ParameterValue("plan"));
  node->get_parameter(name_ + "." + "path_topic", path_topic_);

  // Declare and get parameters
  declareParameter("look_ahead", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + "." + "look_ahead", look_ahead_);

  declareParameter("step", rclcpp::ParameterValue(5));
  node->get_parameter(name_ + "." + "step", step_);
  step_size_ = static_cast<size_t>(step_);

  declareParameter("corridor_width", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + "." + "corridor_width", corridor_width_);

  declareParameter("wall_thickness", rclcpp::ParameterValue(1));
  node->get_parameter(name_ + "." + "wall_thickness", wall_thickness_);

  declareParameter("enabled", rclcpp::ParameterValue(true));
  bool enabled_param;
  node->get_parameter(name_ + "." + "enabled", enabled_param);
  enabled_.store(enabled_param);

  if (wall_thickness_ <= 0) {
    throw std::runtime_error{"wall_thickness must be greater than zero"};
  }

  if (look_ahead_ <= 0.0) {
    throw std::runtime_error{"look_ahead must be positive"};
  }
  if (corridor_width_ <= 0.0) {
    throw std::runtime_error{"corridor_width must be positive"};
  }
  if (step_size_ == 0) {
    throw std::runtime_error{"step must be greater than zero"};
  }
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &BoundedTrackingErrorLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void BoundedTrackingErrorLayer::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  auto now = node->now();
  auto msg_time = rclcpp::Time(msg->header.stamp);
  auto age = (now - msg_time).seconds();

  if (age > 1.0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Path is %.2f seconds old", age);
    return;
  }
  last_path_ = *msg;
  last_tracking_feedback_ = nav2_msgs::msg::TrackingFeedback();
}

void BoundedTrackingErrorLayer::trackingCallback(
  const nav2_msgs::msg::TrackingFeedback::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if timestamp is sufficiently current
  auto now = node->now();
  auto msg_time = rclcpp::Time(msg->header.stamp);
  auto age = (now - msg_time).seconds();

  if (age > 1.0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Tracking feedback is %.2f seconds old", age);
    return;
  }

  last_tracking_feedback_ = *msg;
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

std::vector<std::vector<double>> BoundedTrackingErrorLayer::getWallPoints(
  const nav_msgs::msg::Path & segment)
{
  std::vector<std::vector<double>> point_list;

  // Early return checks
  if (segment.poses.empty() || step_size_ == 0) {
    return point_list;
  }

  // Reserve space for better performance (estimate 2 points per step)
  size_t estimated_points = (segment.poses.size() / step_size_) * 2;
  point_list.reserve(estimated_points);

  // Process path points with step size
  for (size_t current_index = 0; current_index < segment.poses.size();
    current_index += step_size_)
  {
    const auto & current_pose = segment.poses[current_index];
    double px = current_pose.pose.position.x;
    double py = current_pose.pose.position.y;

    // Calculate direction vector to next point
    double dx = 0.0, dy = 0.0;

    if (current_index + step_size_ < segment.poses.size()) {
      // Use next point at step distance
      const auto & next_pose = segment.poses[current_index + step_size_];
      dx = next_pose.pose.position.x - px;
      dy = next_pose.pose.position.y - py;
    } else if (current_index + 1 < segment.poses.size()) {
      // Use immediate next point if step would go out of bounds
      const auto & next_pose = segment.poses[current_index + 1];
      dx = next_pose.pose.position.x - px;
      dy = next_pose.pose.position.y - py;
    } else {
      // Last point - use previous direction if available
      if (current_index > 0) {
        const auto & prev_pose = segment.poses[current_index - 1];
        dx = px - prev_pose.pose.position.x;
        dy = py - prev_pose.pose.position.y;
      } else {
        continue;
      }
    }

    // Calculate perpendicular direction for wall points
    double norm = std::hypot(dx, dy);
    if (norm < 1e-6) {
      continue;
    }

    double perp_x = -dy / norm;
    double perp_y = dx / norm;

    // Multiply by 0.5 to get half the width, creating symmetric boundaries on both sides of the path
    double half_width = corridor_width_ * 0.5;

    point_list.push_back({px + perp_x * half_width, py + perp_y * half_width});
    point_list.push_back({px - perp_x * half_width, py - perp_y * half_width});
  }

  return point_list;
}

nav_msgs::msg::Path BoundedTrackingErrorLayer::getPathSegment()
{
  // NOTE: Caller must hold data_mutex_ before calling this method
  nav_msgs::msg::Path segment;
  if (last_path_.poses.empty() ||
    last_tracking_feedback_.current_path_index >= last_path_.poses.size())
  {
    return segment;
  }

  size_t start_index = last_tracking_feedback_.current_path_index;
  size_t end_index = start_index;
  double dist_traversed = 0.0;

  // Limit the corridor with look_ahead_
  for (size_t i = start_index; i < last_path_.poses.size() - 1; ++i) {
    dist_traversed += nav2_util::geometry_utils::euclidean_distance(
      last_path_.poses[i], last_path_.poses[i + 1]);
    end_index = i + 1;
    if (dist_traversed >= look_ahead_) {
      break;
    }
  }

  // Create the forward corridor segment
  if (start_index < end_index && end_index <= last_path_.poses.size()) {
    segment.header = last_path_.header;
    segment.poses.assign(
      last_path_.poses.begin() + start_index,
      last_path_.poses.begin() + end_index
    );
  } else if (start_index == last_path_.poses.size() - 1) {
    segment.header = last_path_.header;
    segment.poses.push_back(last_path_.poses.back());
  }

  return segment;
}

void BoundedTrackingErrorLayer::drawWallLine(
  nav2_costmap_2d::Costmap2D & master_grid,
  unsigned int x0, unsigned int y0,
  unsigned int x1, unsigned int y1,
  unsigned int map_size_x, unsigned int map_size_y)
{
  // Calculate perpendicular direction for wall thickness
  int dx = static_cast<int>(x1) - static_cast<int>(x0);
  int dy = static_cast<int>(y1) - static_cast<int>(y0);
  double norm = std::hypot(dx, dy);

  if (norm < 1e-6) {
    return;
  }

  double perp_x = -dy / norm;
  double perp_y = dx / norm;

  // Draw parallel lines for thickness
  for (int t = 0; t < wall_thickness_; ++t) {
    int offset_x = static_cast<int>(std::round(perp_x * t));
    int offset_y = static_cast<int>(std::round(perp_y * t));

    // Use signed arithmetic to avoid unsigned underflow
    int thick_x0_signed = static_cast<int>(x0) + offset_x;
    int thick_y0_signed = static_cast<int>(y0) + offset_y;
    int thick_x1_signed = static_cast<int>(x1) + offset_x;
    int thick_y1_signed = static_cast<int>(y1) + offset_y;

    // Skip if any coordinate is negative (would underflow as unsigned)
    if (thick_x0_signed < 0 || thick_y0_signed < 0 ||
      thick_x1_signed < 0 || thick_y1_signed < 0)
    {
      continue;
    }

    unsigned int thick_x0 = static_cast<unsigned int>(thick_x0_signed);
    unsigned int thick_y0 = static_cast<unsigned int>(thick_y0_signed);
    unsigned int thick_x1 = static_cast<unsigned int>(thick_x1_signed);
    unsigned int thick_y1 = static_cast<unsigned int>(thick_y1_signed);

    // Skip if coordinates are outside map bounds
    if (thick_x0 >= map_size_x || thick_y0 >= map_size_y ||
      thick_x1 >= map_size_x || thick_y1 >= map_size_y)
    {
      continue;
    }

    auto cells = nav2_util::geometry_utils::bresenham(thick_x0, thick_y0, thick_x1, thick_y1);
    for (const auto & cell : cells) {
      if (cell[0] < map_size_x && cell[1] < map_size_y) {
        master_grid.setCost(cell[0], cell[1], nav2_costmap_2d::LETHAL_OBSTACLE);
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

  auto node = node_.lock();

  // Quickly copy the data we need under lock, then release
  nav_msgs::msg::Path segment;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    segment = getPathSegment();
  }

  if (segment.poses.size() < 2) {
    if (node) {
      RCLCPP_DEBUG_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        5000,
        "Path segment too small (%zu poses), skipping wall generation",
        segment.poses.size());
    }
    return;
  }

  std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  nav_msgs::msg::Path transformed_segment;
  transformed_segment.header.frame_id = costmap_frame;
  transformed_segment.poses.reserve(segment.poses.size());

  // Use longer timeout for first pose to wait for TF, no wait for rest
  for (size_t i = 0; i < segment.poses.size(); ++i) {
    const auto & pose = segment.poses[i];
    geometry_msgs::msg::PoseStamped transformed_pose;
    double timeout = (i == 0) ? 0.1 : 0.0;
    if (nav2_util::transformPoseInTargetFrame(
        pose, transformed_pose, *tf_, costmap_frame, timeout))
    {
      transformed_segment.poses.push_back(transformed_pose);
    } else {
      if (node) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          5000,
          "Failed to transform pose %zu to %s, skipping wall generation",
          i,
          costmap_frame.c_str());
      }
      return;
    }
  }

  auto wall_points = getWallPoints(transformed_segment);

  // Get map dimensions for bounds checking
  unsigned int map_size_x = master_grid.getSizeInCellsX();
  unsigned int map_size_y = master_grid.getSizeInCellsY();

  // Draw lines between consecutive wall points using Bresenham
  for (size_t i = 0; i < wall_points.size(); i += 2) {
    if (i + 2 < wall_points.size()) {
      unsigned int x0, y0, x1, y1;

      // Line on left side of path
      if (master_grid.worldToMap(wall_points[i][0], wall_points[i][1], x0, y0) &&
        master_grid.worldToMap(wall_points[i + 2][0], wall_points[i + 2][1], x1, y1))
      {
        drawWallLine(master_grid, x0, y0, x1, y1, map_size_x, map_size_y);
      }

      // Line on right side of path
      if (master_grid.worldToMap(wall_points[i + 1][0], wall_points[i + 1][1], x0, y0) &&
        master_grid.worldToMap(wall_points[i + 3][0], wall_points[i + 3][1], x1, y1))
      {
        drawWallLine(master_grid, x0, y0, x1, y1, map_size_x, map_size_y);
      }
    }
  }
}


rcl_interfaces::msg::SetParametersResult BoundedTrackingErrorLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> guard(data_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    // Only process parameters for this layer
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "look_ahead") {
        double new_value = parameter.as_double();
        if (new_value <= 0.0) {
          result.successful = false;
          result.reason = "look_ahead must be positive";
          return result;
        }
        look_ahead_ = new_value;
      } else if (param_name == name_ + "." + "corridor_width") {
        double new_value = parameter.as_double();
        if (new_value <= 0.0) {
          result.successful = false;
          result.reason = "corridor_width must be positive";
          return result;
        }
        corridor_width_ = new_value;
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_.store(parameter.as_bool());
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "step") {
        int new_value = parameter.as_int();
        if (new_value <= 0) {
          result.successful = false;
          result.reason = "step must be greater than zero";
          return result;
        }
        step_ = new_value;
        step_size_ = static_cast<size_t>(step_);
      } else if (param_name == name_ + "." + "wall_thickness") {
        int new_value = parameter.as_int();
        if (new_value <= 0) {
          result.successful = false;
          result.reason = "wall_thickness must be greater than zero";
          return result;
        }
        wall_thickness_ = new_value;
      }
    }
  }

  return result;
}

void BoundedTrackingErrorLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Create subscriptions when layer is activated
  // Join topics with parent namespace for proper namespacing
  std::string path_topic = joinWithParentNamespace(path_topic_);
  std::string tracking_feedback_topic = joinWithParentNamespace(tracking_feedback_topic_);

  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic,
    std::bind(&BoundedTrackingErrorLayer::pathCallback, this, std::placeholders::_1)
  );

  tracking_feedback_sub_ = node->create_subscription<nav2_msgs::msg::TrackingFeedback>(
    tracking_feedback_topic,
    std::bind(&BoundedTrackingErrorLayer::trackingCallback, this, std::placeholders::_1)
  );

  enabled_.store(true);
  current_ = true;
}

void BoundedTrackingErrorLayer::deactivate()
{
  // Destroy subscriptions when layer is deactivated
  path_sub_.reset();
  tracking_feedback_sub_.reset();

  // Clear cached data
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_path_ = nav_msgs::msg::Path();
    last_tracking_feedback_ = nav2_msgs::msg::TrackingFeedback();
  }

  enabled_.store(false);
}

void BoundedTrackingErrorLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_path_ = nav_msgs::msg::Path();
  current_ = true;
}

}  // namespace nav2_costmap_2d
