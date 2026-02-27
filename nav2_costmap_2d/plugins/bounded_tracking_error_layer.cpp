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

  tracking_feedback_topic_ = node->declare_or_get_parameter(
    name_ + "." + "tracking_feedback_topic", std::string("tracking_feedback"));

  path_topic_ = node->declare_or_get_parameter(
    name_ + "." + "path_topic", std::string("plan"));

  look_ahead_ = node->declare_or_get_parameter(name_ + "." + "look_ahead", 3.0);

  step_ = node->declare_or_get_parameter(name_ + "." + "step", 5);
  step_size_ = static_cast<size_t>(step_);

  corridor_width_ = node->declare_or_get_parameter(name_ + "." + "corridor_width", 2.0);

  wall_thickness_ = node->declare_or_get_parameter(name_ + "." + "wall_thickness", 1);

  int corridor_cost_param = node->declare_or_get_parameter(name_ + "." + "corridor_cost", 190);
  corridor_cost_ = static_cast<unsigned char>(std::clamp(corridor_cost_param, 1, 254));

  bool enabled_param = node->declare_or_get_parameter(name_ + "." + "enabled", true);
  enabled_.store(enabled_param);

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

  // Join topics with parent namespace for proper namespacing
  std::string path_topic = joinWithParentNamespace(path_topic_);
  std::string tracking_feedback_topic = joinWithParentNamespace(tracking_feedback_topic_);

  // Create subscriptions in onInitialize like other layers
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

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &BoundedTrackingErrorLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
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
    double dx = msg->pose.position.x - last_goal_.pose.position.x;
    double dy = msg->pose.position.y - last_goal_.pose.position.y;
    double distance = std::hypot(dx, dy);

    if (distance > 0.1) {
      is_new_goal = true;
    }
  }

  if (is_new_goal) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "New goal received, clearing corridor");

    last_path_ = nav_msgs::msg::Path();
    last_tracking_feedback_ = nav2_msgs::msg::TrackingFeedback();
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
  auto now = node->now();
  auto msg_time = rclcpp::Time(msg->header.stamp);
  auto age = (now - msg_time).seconds();

  if (age > 2.0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Path is %.2f seconds old, clearing corridor", age);
    last_path_ = nav_msgs::msg::Path();
    return;
  }

  // Check for path discontinuity (replanning detection)
  // Discontinuity indicates the planner has generated a significantly different path,
  // requiring corridor to be rebuilt from scratch
  bool path_discontinuity = false;
  if (!last_path_.poses.empty() && msg->poses.size() >= 2) {
    size_t old_size = last_path_.poses.size();
    size_t new_size = msg->poses.size();

    if (old_size != new_size) {
      path_discontinuity = true;
      RCLCPP_DEBUG(
        node->get_logger(),
        "Path size changed from %zu to %zu, clearing corridor", old_size, new_size);
    } else if (old_size >= 2) {
      double threshold = corridor_width_ * 0.5;

      // Compare midpoint and endpoint positions between old and new paths
      // to detect significant path changes even when size remains the same
      size_t old_mid_idx = old_size / 2;
      size_t new_mid_idx = new_size / 2;
      double mid_dx = msg->poses[new_mid_idx].pose.position.x -
        last_path_.poses[old_mid_idx].pose.position.x;
      double mid_dy = msg->poses[new_mid_idx].pose.position.y -
        last_path_.poses[old_mid_idx].pose.position.y;
      double mid_dist = std::hypot(mid_dx, mid_dy);

      double end_dx = msg->poses.back().pose.position.x -
        last_path_.poses.back().pose.position.x;
      double end_dy = msg->poses.back().pose.position.y -
        last_path_.poses.back().pose.position.y;
      double end_dist = std::hypot(end_dx, end_dy);

      if (mid_dist > threshold || end_dist > threshold) {
        path_discontinuity = true;
        RCLCPP_DEBUG(
          node->get_logger(),
          "Path discontinuity detected (mid: %.2fm, end: %.2fm), clearing corridor",
          mid_dist, end_dist);
      }
    }
  }

  if (path_discontinuity) {
    last_tracking_feedback_ = nav2_msgs::msg::TrackingFeedback();
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

WallPolygons BoundedTrackingErrorLayer::getWallPolygons(
  const nav_msgs::msg::Path & segment)
{
  WallPolygons walls;

  if (segment.poses.empty() || step_size_ == 0) {
    return walls;
  }

  // Get the costmap resolution for calculating inner width
  double resolution = layered_costmap_->getCostmap()->getResolution();

  double outer_half_width = corridor_width_ * 0.5;
  double inner_half_width = (corridor_width_ - (wall_thickness_ * resolution * 2.0)) * 0.5;

  // Math: Let n_hat = (-dy/|d|, dx/|d|) be the unit normal to path segment d.
  // Left Wall:  P_out = Pi + n_hat * (width/2), P_in = Pi + n_hat * (width/2 - thickness * res)
  // Right Wall: P_out = Pi - n_hat * (width/2), P_in = Pi - n_hat * (width/2 - thickness * res)

  size_t estimated_points = (segment.poses.size() / step_size_) + 1;
  walls.left_outer.reserve(estimated_points);
  walls.left_inner.reserve(estimated_points);
  walls.right_outer.reserve(estimated_points);
  walls.right_inner.reserve(estimated_points);

  for (size_t current_index = 0; current_index < segment.poses.size();
    current_index += step_size_)
  {
    const auto & current_pose = segment.poses[current_index];
    double px = current_pose.pose.position.x;
    double py = current_pose.pose.position.y;

    if (current_index + step_size_ >= segment.poses.size()) {
      break;
    }

    const auto & next_pose = segment.poses[current_index + step_size_];
    double dx = next_pose.pose.position.x - px;
    double dy = next_pose.pose.position.y - py;

    // Calculate perpendicular direction for wall points
    // Given direction vector (dx, dy), the perpendicular is (-dy, dx) normalized
    double norm = std::hypot(dx, dy);
    if (norm < 1e-6) {  // Avoid division by zero for very close points
      continue;
    }

    double perp_x = -dy / norm;
    double perp_y = dx / norm;

    walls.left_outer.push_back({px + perp_x * outer_half_width, py + perp_y * outer_half_width});
    walls.left_inner.push_back({px + perp_x * inner_half_width, py + perp_y * inner_half_width});
    walls.right_outer.push_back({px - perp_x * outer_half_width, py - perp_y * outer_half_width});
    walls.right_inner.push_back({px - perp_x * inner_half_width, py - perp_y * inner_half_width});
  }

  return walls;
}

nav_msgs::msg::Path BoundedTrackingErrorLayer::getPathSegment()
{
  nav_msgs::msg::Path segment;
  if (last_path_.poses.empty() ||
    last_tracking_feedback_.current_path_index >= last_path_.poses.size())
  {
    return segment;
  }

  size_t start_index = last_tracking_feedback_.current_path_index;
  size_t end_index = start_index;
  double dist_traversed = 0.0;

  for (size_t i = start_index; i < last_path_.poses.size() - 1; ++i) {
    dist_traversed += nav2_util::geometry_utils::euclidean_distance(
      last_path_.poses[i], last_path_.poses[i + 1]);
    end_index = i + 1;
    if (dist_traversed >= look_ahead_) {
      break;
    }
  }

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

void BoundedTrackingErrorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_.load()) {
    return;
  }

  auto node = node_.lock();

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

  // Use longer timeout for first pose to wait for TF
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

  auto walls = getWallPolygons(transformed_segment);

  if (walls.left_outer.empty() || walls.right_outer.empty()) {
    if (node) {
      RCLCPP_DEBUG_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        5000,
        "Not enough wall points to form polygons");
    }
    return;
  }

  // Build left wall polygon by connecting outer and inner edges
  // Creates a closed polygon: outer edge forward, inner edge backward
  std::vector<geometry_msgs::msg::Point> left_wall_polygon;
  left_wall_polygon.reserve(walls.left_outer.size() + walls.left_inner.size());

  for (const auto & pt : walls.left_outer) {
    geometry_msgs::msg::Point point;
    point.x = pt[0];
    point.y = pt[1];
    point.z = 0.0;
    left_wall_polygon.push_back(point);
  }

  for (auto it = walls.left_inner.rbegin(); it != walls.left_inner.rend(); ++it) {
    geometry_msgs::msg::Point point;
    point.x = (*it)[0];
    point.y = (*it)[1];
    point.z = 0.0;
    left_wall_polygon.push_back(point);
  }

  // Build right wall polygon similarly
  std::vector<geometry_msgs::msg::Point> right_wall_polygon;
  right_wall_polygon.reserve(walls.right_outer.size() + walls.right_inner.size());

  for (const auto & pt : walls.right_outer) {
    geometry_msgs::msg::Point point;
    point.x = pt[0];
    point.y = pt[1];
    point.z = 0.0;
    right_wall_polygon.push_back(point);
  }

  for (auto it = walls.right_inner.rbegin(); it != walls.right_inner.rend(); ++it) {
    geometry_msgs::msg::Point point;
    point.x = (*it)[0];
    point.y = (*it)[1];
    point.z = 0.0;
    right_wall_polygon.push_back(point);
  }

  master_grid.setConvexPolygonCost(left_wall_polygon, corridor_cost_);
  master_grid.setConvexPolygonCost(right_wall_polygon, corridor_cost_);
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

    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "look_ahead") {
        double new_value = parameter.as_double();
        if (new_value < 0.0) {
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
      } else if (param_name == name_ + "." + "corridor_cost") {
        int new_value = parameter.as_int();
        if (new_value <= 0 || new_value > 254) {
          result.successful = false;
          result.reason = "corridor_cost must be between 1 and 254";
          return result;
        }
        corridor_cost_ = static_cast<unsigned char>(new_value);
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
  enabled_.store(true);
  current_ = true;
}

void BoundedTrackingErrorLayer::deactivate()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_path_ = nav_msgs::msg::Path();
    last_tracking_feedback_ = nav2_msgs::msg::TrackingFeedback();
    last_goal_ = geometry_msgs::msg::PoseStamped();
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
