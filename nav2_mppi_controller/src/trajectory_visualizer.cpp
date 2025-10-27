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

  auto getParam = parameters_handler->getParamGetter(name + ".Visualization");

  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);
  getParam(publish_optimal_trajectory_, "publish_optimal_trajectory", false);
  getParam(publish_trajectories_with_total_cost_, "publish_trajectories_with_total_cost", false);
  getParam(publish_trajectories_with_individual_cost_, "publish_trajectories_with_individual_cost", false);

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
  const models::Trajectories & trajectories,
  const Eigen::ArrayXf & total_costs,
  const std::vector<std::pair<std::string, Eigen::ArrayXf>> & individual_critics_cost,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  // Check if we should visualize per-critic costs
  bool visualize_per_critic = !individual_critics_cost.empty() && 
                               publish_trajectories_with_individual_cost_ &&
                               trajectories_publisher_->get_subscription_count() > 0;

  size_t n_rows = trajectories.x.rows();
  size_t n_cols = trajectories.x.cols();
  points_->markers.reserve(n_rows / trajectory_step_);

  // Helper lambda to create a trajectory marker with given costs and namespace
  auto create_trajectory_markers = [&](
    const Eigen::ArrayXf & costs,
    const std::string & ns,
    bool use_collision_coloring) {
    
    // Find min/max cost for normalization
    float min_cost = std::numeric_limits<float>::max();
    float max_cost = std::numeric_limits<float>::lowest();
    
    for (Eigen::Index i = 0; i < costs.size(); ++i) {
      // Skip collision trajectories for min/max calculation if using collision coloring
      if (use_collision_coloring && costs(i) >= 1000000.0f) {
        continue;
      }
      min_cost = std::min(min_cost, costs(i));
      max_cost = std::max(max_cost, costs(i));
    }
    
    float cost_range = max_cost - min_cost;
    
    // Avoid division by zero
    if (cost_range < 1e-6f) {
      cost_range = 1.0f;
    }

    for (size_t i = 0; i < n_rows; i += trajectory_step_) {
      float red_component, green_component, blue_component;

      // Check if this trajectory is in collision (cost >= 1000000)
      bool in_collision = use_collision_coloring && costs(i) >= 1000000.0f;
      
      // Check if cost is zero (no contribution from this critic)
      bool zero_cost = std::abs(costs(i)) < 1e-6f;
      
      if (in_collision) {
        // Fixed red color for collision trajectories
        red_component = 1.0f;
        green_component = 0.0f;
        blue_component = 0.0f;
      } else if (zero_cost) {
        // Gray color for zero cost (no contribution)
        red_component = 0.5f;
        green_component = 0.5f;
        blue_component = 0.5f;
      } else {
        // Normal gradient for non-collision trajectories
        float normalized_cost = (costs(i) - min_cost) / cost_range;
        normalized_cost = std::clamp(normalized_cost, 0.0f, 1.0f);

        // Color scheme: Green (low cost) -> Yellow -> Red (high cost)
        blue_component = 0.0f;
        if (normalized_cost < 0.5f) {
          // Green to Yellow (0.0 - 0.5)
          red_component = normalized_cost * 2.0f;
          green_component = 1.0f;
        } else {
          // Yellow to Red (0.5 - 1.0)
          red_component = 1.0f;
          green_component = 2.0f * (1.0f - normalized_cost);
        }
      }

      // Create line strip marker for this trajectory
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = cmd_stamp;
      marker.ns = ns;
      marker.id = marker_id_++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;  // Line width
      marker.color.r = red_component;
      marker.color.g = green_component;
      marker.color.b = blue_component;
      marker.color.a = 1.0;

      // Add all points in this trajectory to the line strip
      for (size_t j = 0; j < n_cols; j += time_step_) {
        geometry_msgs::msg::Point point;
        point.x = trajectories.x(i, j);
        point.y = trajectories.y(i, j);
        point.z = 0.0f;
        marker.points.push_back(point);
      }
      
      points_->markers.push_back(marker);
    }
  };

  // If visualizing per-critic costs
  if (visualize_per_critic) {
    // Visualize total costs if requested
    if (publish_trajectories_with_total_cost_) {
      create_trajectory_markers(total_costs, "Total Costs", false);
    }

    // Visualize each critic's contribution
    for (const auto & [critic_name, costs] : individual_critics_cost) {
      create_trajectory_markers(costs, critic_name, true);  // Use collision coloring
    }
  } else {
    // Simple visualization with just total costs
    create_trajectory_markers(total_costs, "Total Costs", false);
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

  if (publish_optimal_trajectory_ && optimal_path_pub_->get_subscription_count() > 0) {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }

  reset();

  if (transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

}  // namespace mppi
