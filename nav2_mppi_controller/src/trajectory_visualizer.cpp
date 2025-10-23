// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <memory>
#include <vector>
#include <algorithm>
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"

namespace mppi
{

void TrajectoryVisualizer::on_configure(
  nav2::LifecycleNode::WeakPtr parent, const std::string & name,
  const std::string & frame_id, ParametersHandler * parameters_handler)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  frame_id_ = frame_id;
  trajectories_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("~/candidate_trajectories");
  transformed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "~/transformed_global_plan");
  optimal_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/optimal_path");
  parameters_handler_ = parameters_handler;

  auto getParam = parameters_handler->getParamGetter(name + ".TrajectoryVisualizer");

  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);
  getParam(time_step_elevation_, "time_step_elevation", 0.0f);

  reset();
}

void TrajectoryVisualizer::on_cleanup()
{
  trajectories_publisher_.reset();
  transformed_path_pub_.reset();
  optimal_path_pub_.reset();
}

void TrajectoryVisualizer::on_activate()
{
  trajectories_publisher_->on_activate();
  transformed_path_pub_->on_activate();
  optimal_path_pub_->on_activate();
}

void TrajectoryVisualizer::on_deactivate()
{
  trajectories_publisher_->on_deactivate();
  transformed_path_pub_->on_deactivate();
  optimal_path_pub_->on_deactivate();
}

void TrajectoryVisualizer::add(
  const Eigen::ArrayXXf & trajectory,
  const std::string & marker_namespace,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  size_t size = trajectory.rows();
  if (!size) {
    return;
  }

  auto add_marker = [&](auto i) {
      float component = static_cast<float>(i) / static_cast<float>(size);

      auto pose = utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      auto scale =
        i != size - 1 ?
        utils::createScale(0.03, 0.03, 0.07) :
        utils::createScale(0.07, 0.07, 0.09);
      auto color = utils::createColor(0, component, component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);
      points_->markers.push_back(marker);

      // populate optimal path
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = frame_id_;
      pose_stamped.pose = pose;

      tf2::Quaternion quaternion_tf2;
      quaternion_tf2.setRPY(0., 0., trajectory(i, 2));
      pose_stamped.pose.orientation = tf2::toMsg(quaternion_tf2);

      optimal_path_->poses.push_back(pose_stamped);
    };

  optimal_path_->header.stamp = cmd_stamp;
  optimal_path_->header.frame_id = frame_id_;
  for (size_t i = 0; i < size; i++) {
    add_marker(i);
  }
}

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories, const Eigen::ArrayXf & costs,
  const std::string & marker_namespace,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  size_t n_rows = trajectories.x.rows();
  size_t n_cols = trajectories.x.cols();
  points_->markers.reserve(n_rows / trajectory_step_);

  // Use percentile-based normalization to handle outliers
  // Sort costs to find percentiles
  std::vector<float> sorted_costs(costs.data(), costs.data() + costs.size());
  std::sort(sorted_costs.begin(), sorted_costs.end());

  // Use 10th and 90th percentile for robust color mapping
  size_t idx_10th = static_cast<size_t>(sorted_costs.size() * 0.1);
  size_t idx_90th = static_cast<size_t>(sorted_costs.size() * 0.9);

  float min_cost = sorted_costs[idx_10th];
  float max_cost = sorted_costs[idx_90th];
  float cost_range = max_cost - min_cost;

  // Avoid division by zero
  if (cost_range < 1e-6f) {
    cost_range = 1.0f;
  }

  for (size_t i = 0; i < n_rows; i += trajectory_step_) {
    float red_component, green_component, blue_component;

    // Normalize cost using percentile-based range, clamping outliers
    float normalized_cost = (costs(i) - min_cost) / cost_range;

    // Clamp to [0, 1] range (handles outliers beyond percentiles)
    normalized_cost = std::max(0.0f, std::min(1.0f, normalized_cost));

    // Apply power function for better visual distribution
    normalized_cost = std::pow(normalized_cost, 0.5f);

    // Color scheme with smooth gradient:
    // Green (0.0) -> Yellow-Green (0.25) -> Yellow (0.5) -> Orange (0.75) -> Red (1.0)
    // Very high outlier costs (>95th percentile) will be clamped to red
    blue_component = 0.0f;

    if (normalized_cost < 0.5f) {
      // Transition from Green to Yellow (0.0 - 0.5)
      float t = normalized_cost * 2.0f;  // Scale to [0, 1]
      red_component = t;
      green_component = 1.0f;
    } else {
      // Transition from Yellow to Red (0.5 - 1.0)
      float t = (normalized_cost - 0.5f) * 2.0f;  // Scale to [0, 1]
      red_component = 1.0f;
      green_component = 1.0f - t;
    }

    // Create line strip marker for this trajectory
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = cmd_stamp;
    marker.ns = marker_namespace;
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // Set line width
    marker.scale.x = 0.01;  // Line width

    // Set color for entire trajectory
    marker.color.r = red_component;
    marker.color.g = green_component;
    marker.color.b = blue_component;
    marker.color.a = 0.8f;  // Slightly transparent

    // Add all points in this trajectory to the line strip
    for (size_t j = 0; j < n_cols; j += time_step_) {
      geometry_msgs::msg::Point point;
      point.x = trajectories.x(i, j);
      point.y = trajectories.y(i, j);
      // Increment z by time_step_elevation_ for each time step
      if (time_step_elevation_ > 0.0f) {
        point.z = static_cast<float>(j) * time_step_elevation_;
      } else {
        point.z = 0.0f;
      }
      marker.points.push_back(point);
    }

    points_->markers.push_back(marker);
  }
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
  optimal_path_ = std::make_unique<nav_msgs::msg::Path>();
}

void TrajectoryVisualizer::visualize(const nav_msgs::msg::Path & plan)
{
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  if (optimal_path_pub_->get_subscription_count() > 0) {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }

  reset();

  if (transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

}  // namespace mppi
