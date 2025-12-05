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

#include "nav2_costmap_2d/tracking_error_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::TrackingErrorLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

TrackingErrorLayer::TrackingErrorLayer() {}

TrackingErrorLayer::~TrackingErrorLayer()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void TrackingErrorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Declare and get parameters
  declareParameter("look_ahead", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + "." + "look_ahead", look_ahead_);

  declareParameter("step", rclcpp::ParameterValue(5));
  node->get_parameter(name_ + "." + "step", step_);
  temp_step_ = static_cast<size_t>(step_);

  declareParameter("corridor_width", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + "." + "corridor_width", width_);

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  if (look_ahead_ <= 0.0) {
    throw std::runtime_error{"look_ahead must be positive"};
  }
  if (width_ <= 0.0) {
    throw std::runtime_error{"corridor_width must be positive"};
  }
  if (temp_step_ == 0) {
    throw std::runtime_error{"step must be greater than zero"};
  }
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &TrackingErrorLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void TrackingErrorLayer::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_path_ = *msg;
}

void TrackingErrorLayer::trackingCallback(
  const nav2_msgs::msg::TrackingFeedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_tracking_feedback_ = *msg;
}

void TrackingErrorLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  *min_x = std::min(*min_x, robot_x - look_ahead_);
  *max_x = std::max(*max_x, robot_x + look_ahead_);
  *min_y = std::min(*min_y, robot_y - look_ahead_);
  *max_y = std::max(*max_y, robot_y + look_ahead_);
}

std::vector<std::vector<double>> TrackingErrorLayer::getWallPoints(
  const nav_msgs::msg::Path & segment)
{
  std::vector<std::vector<double>> point_list;

  // Early return checks
  if (segment.poses.empty() || temp_step_ == 0) {
    return point_list;
  }

  // Reserve space for better performance (estimate 2 points per step)
  size_t estimated_points = (segment.poses.size() / temp_step_) * 2;
  point_list.reserve(estimated_points);

  // Process path points with step size
  for (size_t current_index = 0; current_index < segment.poses.size();
    current_index += temp_step_)
  {
    const auto & current_pose = segment.poses[current_index];
    double px = current_pose.pose.position.x;
    double py = current_pose.pose.position.y;

    // Calculate direction vector to next point
    double dx = 0.0, dy = 0.0;

    if (current_index + temp_step_ < segment.poses.size()) {
      // Use next point at step distance
      const auto & next_pose = segment.poses[current_index + temp_step_];
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

    double half_width = width_ * 0.5;

    point_list.push_back({px + perp_x * half_width, py + perp_y * half_width});
    point_list.push_back({px - perp_x * half_width, py - perp_y * half_width});
  }

  return point_list;
}


nav_msgs::msg::Path TrackingErrorLayer::getPathSegment()
{
  std::lock_guard<std::mutex> path_lock(data_mutex_);

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

void TrackingErrorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::Path segment = getPathSegment();
  if (segment.poses.size() < 2) {
    return;
  }

  std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  nav_msgs::msg::Path transformed_segment;
  transformed_segment.header.frame_id = costmap_frame;

  for (const auto & pose : segment.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (nav2_util::transformPoseInTargetFrame(
          pose, transformed_pose, *tf_, costmap_frame, 0.1))
    {
      transformed_segment.poses.push_back(transformed_pose);
    }
  }

  auto wall_points = getWallPoints(transformed_segment);

  // Create 3x3 blocks around each wall point
  for (const auto & point : wall_points) {
    double wx = point[0];
    double wy = point[1];

    unsigned int center_x, center_y;
    if (master_grid.worldToMap(wx, wy, center_x, center_y)) {
      // Create 3x3 block centered on the point
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          unsigned int map_x = center_x + dx;
          unsigned int map_y = center_y + dy;

          // Check bounds before setting cost
          if (map_x < master_grid.getSizeInCellsX() &&
            map_y < master_grid.getSizeInCellsY())
          {
            master_grid.setCost(map_x, map_y, nav2_costmap_2d::LETHAL_OBSTACLE);
          }
        }
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult TrackingErrorLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
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
        width_ = new_value;
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
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
        temp_step_ = static_cast<size_t>(step_);
      }
    }
  }

  return result;
}

void TrackingErrorLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Create subscriptions when layer is activated
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/plan",
    std::bind(&TrackingErrorLayer::pathCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS()
  );

  tracking_feedback_sub_ = node->create_subscription<nav2_msgs::msg::TrackingFeedback>(
    "/tracking_feedback",
    std::bind(&TrackingErrorLayer::trackingCallback, this, std::placeholders::_1),
    nav2::qos::StandardTopicQoS()
  );

  enabled_ = true;
  current_ = true;
}

void TrackingErrorLayer::deactivate()
{
  // Destroy subscriptions when layer is deactivated
  path_sub_.reset();
  tracking_feedback_sub_.reset();

  // Clear cached data
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_path_.poses.clear();
    last_path_.header = std_msgs::msg::Header();
    last_tracking_feedback_ = nav2_msgs::msg::TrackingFeedback();
  }

  enabled_ = false;
}

void TrackingErrorLayer::reset()
{
  current_ = true;
}

}  // namespace nav2_costmap_2d
