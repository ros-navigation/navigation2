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
  parameters_handler_ = parameters_handler;
  auto getParam = parameters_handler->getParamGetter(name + ".Visualization");
  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);
  getParam(publish_optimal_trajectory_, "publish_optimal_trajectory", false);
  getParam(publish_trajectories_with_total_cost_, "publish_trajectories_with_total_cost", false);
  getParam(publish_trajectories_with_individual_cost_, "publish_trajectories_with_individual_cost",
      false);
  getParam(publish_optimal_footprints_, "publish_optimal_footprints", false);
  getParam(publish_optimal_trajectory_msg_, "publish_optimal_trajectory_msg", false);
  getParam(publish_transformed_path_, "publish_transformed_path", false);
  getParam(publish_optimal_path_, "publish_optimal_path", false);
  getParam(footprint_downsample_factor_, "footprint_downsample_factor", 3);

  if (publish_trajectories_with_total_cost_ || publish_trajectories_with_individual_cost_) {
    trajectories_publisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("~/candidate_trajectories");
  }
  if (publish_transformed_path_) {
    transformed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "~/transformed_global_plan");
  }
  if (publish_optimal_path_) {
    optimal_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/optimal_path");
  }
  if (publish_optimal_footprints_) {
    optimal_footprints_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/optimal_footprints");
  }
  if (publish_optimal_trajectory_msg_) {
    optimal_trajectory_msg_pub_ = node->create_publisher<nav2_msgs::msg::Trajectory>(
      "~/optimal_trajectory");
  }

  reset();
}

void TrajectoryVisualizer::on_cleanup()
{
  trajectories_publisher_.reset();
  transformed_path_pub_.reset();
  optimal_path_pub_.reset();
  optimal_footprints_pub_.reset();
  optimal_trajectory_msg_pub_.reset();
}

void TrajectoryVisualizer::on_activate()
{
  if (trajectories_publisher_) {
    trajectories_publisher_->on_activate();
  }
  if (transformed_path_pub_) {
    transformed_path_pub_->on_activate();
  }
  if (optimal_path_pub_) {
    optimal_path_pub_->on_activate();
  }
  if (optimal_footprints_pub_) {
    optimal_footprints_pub_->on_activate();
  }
  if (optimal_trajectory_msg_pub_) {
    optimal_trajectory_msg_pub_->on_activate();
  }
}

void TrajectoryVisualizer::on_deactivate()
{
  if (trajectories_publisher_) {
    trajectories_publisher_->on_deactivate();
  }
  if (transformed_path_pub_) {
    transformed_path_pub_->on_deactivate();
  }
  if (optimal_path_pub_) {
    optimal_path_pub_->on_deactivate();
  }
  if (optimal_footprints_pub_) {
    optimal_footprints_pub_->on_deactivate();
  }
  if (optimal_trajectory_msg_pub_) {
    optimal_trajectory_msg_pub_->on_deactivate();
  }
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
    trajectories_publisher_ && trajectories_publisher_->get_subscription_count() > 0;

  size_t n_rows = trajectories.x.rows();
  points_->markers.reserve(n_rows / trajectory_step_);

  // Visualize total costs if requested
  if (publish_trajectories_with_total_cost_) {
    createTrajectoryMarkers(trajectories, total_costs, "Total Costs", cmd_stamp);
  }

  // Visualize each critic's contribution if requested
  if (visualize_per_critic) {
    for (const auto & [critic_name, costs] : individual_critics_cost) {
      createTrajectoryMarkers(trajectories, costs, critic_name, cmd_stamp);
    }
  }
}

void TrajectoryVisualizer::createTrajectoryMarkers(
  const models::Trajectories & trajectories,
  const Eigen::ArrayXf & costs,
  const std::string & ns,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  size_t n_rows = trajectories.x.rows();
  size_t n_cols = trajectories.x.cols();

  // Find min/max cost for normalization
  float min_cost = std::numeric_limits<float>::max();
  float max_cost = std::numeric_limits<float>::lowest();

  for (Eigen::Index i = 0; i < costs.size(); ++i) {
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

    // Check if cost is zero (no contribution from this critic)
    bool zero_cost = std::abs(costs(i)) < 1e-6f;

    if (zero_cost) {
      // Gray color for zero cost (no contribution)
      red_component = 0.5f;
      green_component = 0.5f;
      blue_component = 0.5f;
    } else {
      // Normal gradient for trajectories
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
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
  optimal_path_ = std::make_unique<nav_msgs::msg::Path>();
}

void TrajectoryVisualizer::visualize(
  nav_msgs::msg::Path plan,
  const Eigen::ArrayXXf & optimal_trajectory,
  const models::ControlSequence & control_sequence,
  float model_dt,
  const builtin_interfaces::msg::Time & stamp,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const models::Trajectories & candidate_trajectories,
  const Eigen::ArrayXf & costs,
  const std::vector<std::pair<std::string, Eigen::ArrayXf>> & critic_costs)
{
  // Create header with frame from costmap
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = costmap_ros->getGlobalFrameID();

  // Visualize trajectories with total costs
  if (publish_trajectories_with_total_cost_ ||
    (!publish_trajectories_with_individual_cost_ || critic_costs.empty()))
  {
    add(candidate_trajectories, costs, {}, stamp);
  }

  // Visualize trajectories with individual critic costs
  if (publish_trajectories_with_individual_cost_ && !critic_costs.empty()) {
    add(candidate_trajectories, costs, critic_costs, stamp);
  }

  // Add optimal trajectory to populate optimal_path_
  if (publish_optimal_trajectory_ && optimal_trajectory.rows() > 0) {
    add(optimal_trajectory, "Optimal Trajectory", stamp);
  }

  // Publish trajectories
  if (trajectories_publisher_ && trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  // Publish optimal path if enabled
  if (publish_optimal_trajectory_ && optimal_path_pub_ &&
    optimal_path_pub_->get_subscription_count() > 0)
  {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }

  // Publish optimal footprints if enabled
  if (publish_optimal_footprints_ && optimal_footprints_pub_ &&
    optimal_footprints_pub_->get_subscription_count() > 0 &&
    costmap_ros != nullptr && optimal_trajectory.rows() > 0)
  {
    auto footprints_msg = createFootprintMarkers(optimal_trajectory, header, costmap_ros);
    optimal_footprints_pub_->publish(std::move(footprints_msg));
  }

  // Publish optimal trajectory message if enabled
  if (publish_optimal_trajectory_msg_ && optimal_trajectory_msg_pub_ &&
    optimal_trajectory_msg_pub_->get_subscription_count() > 0)
  {
    auto trajectory_msg = utils::toTrajectoryMsg(
      optimal_trajectory,
      control_sequence,
      model_dt,
      header);
    optimal_trajectory_msg_pub_->publish(std::move(trajectory_msg));
  }

  reset();

  // Publish transformed path
  if (transformed_path_pub_ && transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

void TrajectoryVisualizer::visualize(nav_msgs::msg::Path plan)
{
  // Simplified version for testing that only publishes what's been added
  if (trajectories_publisher_ && trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  if (publish_optimal_trajectory_ && optimal_path_pub_ &&
    optimal_path_pub_->get_subscription_count() > 0)
  {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }

  reset();

  if (transformed_path_pub_ && transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

visualization_msgs::msg::MarkerArray TrajectoryVisualizer::createFootprintMarkers(
  const Eigen::ArrayXXf & trajectory,
  const std_msgs::msg::Header & header,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (trajectory.rows() == 0) {
    return marker_array;
  }

  // Get robot footprint
  std::vector<geometry_msgs::msg::Point> robot_footprint =
    costmap_ros->getRobotFootprint();

  // Skip if footprint is empty or very small
  if (robot_footprint.size() < 3) {
    return marker_array;
  }

  // Create header for markers
  std_msgs::msg::Header costmap_header;
  costmap_header.frame_id = costmap_ros->getGlobalFrameID();
  costmap_header.stamp = header.stamp;

  int marker_id = 0;
  for (int i = 0; i < trajectory.rows(); i += footprint_downsample_factor_) {
    double x = trajectory(i, 0);
    double y = trajectory(i, 1);
    double theta = trajectory(i, 2);

    // Create oriented footprint
    geometry_msgs::msg::PolygonStamped oriented_footprint;
    oriented_footprint.header = costmap_header;
    nav2_costmap_2d::transformFootprint(x, y, theta, robot_footprint, oriented_footprint);
    // Create marker for this footprint
    visualization_msgs::msg::Marker marker;
    marker.header = costmap_header;
    marker.ns = "optimal_footprints";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // Set marker scale and color
    marker.scale.x = 0.02;  // Line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    // Add footprint points to marker
    for (const auto & point : oriented_footprint.polygon.points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    // Close the polygon by adding the first point again
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points[0]);
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

}  // namespace mppi
