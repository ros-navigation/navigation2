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
#include <algorithm>
#include <cmath>
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
  optimal_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/optimal_path");
  parameters_handler_ = parameters_handler;

  auto getParam = parameters_handler->getParamGetter(name + ".TrajectoryVisualizer");

  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);

  reset();
}

void TrajectoryVisualizer::on_cleanup()
{
  trajectories_publisher_.reset();
  optimal_path_pub_.reset();
}

void TrajectoryVisualizer::on_activate()
{
  trajectories_publisher_->on_activate();
  optimal_path_pub_->on_activate();
}

void TrajectoryVisualizer::on_deactivate()
{
  trajectories_publisher_->on_deactivate();
  optimal_path_pub_->on_deactivate();
}

void TrajectoryVisualizer::add(
  const Eigen::ArrayXXf & trajectory,
  const std::string & marker_namespace,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  if (optimal_path_pub_->get_subscription_count() == 0 &&
    trajectories_publisher_->get_subscription_count() == 0)
  {
    return;
  }

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
  const Eigen::ArrayXf & costs,
  const std::vector<bool> & collisions,
  const builtin_interfaces::msg::Time & stamp)
{
  if (trajectories_publisher_->get_subscription_count() == 0) {
    return;
  }

  const size_t n_rows = trajectories.x.rows();
  if (n_rows == 0 || costs.size() == 0 ||
    static_cast<size_t>(costs.size()) < n_rows)
  {
    return;
  }

  // Normalize costs excluding collision trajectories for better gradient resolution.
  float min_val = costs.maxCoeff();
  float max_val = costs.minCoeff();
  for (Eigen::Index k = 0; k < costs.size(); ++k) {
    if (!collisions.empty() && collisions[k]) {continue;}
    if (costs(k) < min_val) {min_val = costs(k);}
    if (costs(k) > max_val) {max_val = costs(k);}
  }
  if (max_val < min_val) {
    min_val = costs.minCoeff();
    max_val = costs.maxCoeff();
  }
  float range = max_val - min_val;

  for (size_t i = 0; i < n_rows; i += trajectory_step_) {
    float norm = (range > 0.0f) ?
      (costs(i) - min_val) / range : 0.0f;
    bool in_collision =
      !collisions.empty() && i < collisions.size() && collisions[i];
    addCostColoredTrajectory(i, trajectories, norm, in_collision, stamp);
  }
}

void TrajectoryVisualizer::addCostColoredTrajectory(
  size_t trajectory_idx,
  const models::Trajectories & trajectories,
  float normalized_cost,
  bool in_collision,
  const builtin_interfaces::msg::Time & stamp)
{
  using visualization_msgs::msg::Marker;
  const size_t n_cols = trajectories.x.cols();

  Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp;
  marker.ns = "Candidate Trajectories";
  marker.id = marker_id_++;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;  // line width
  marker.color = in_collision ?
    utils::createColor(1.0f, 0.0f, 1.0f, 0.6f) :  // magenta for collisions
    costToColor(normalized_cost);

  marker.points.reserve(n_cols / time_step_ + 1);
  for (size_t j = 0; j < n_cols; j += time_step_) {
    geometry_msgs::msg::Point pt;
    pt.x = trajectories.x(trajectory_idx, j);
    pt.y = trajectories.y(trajectory_idx, j);
    pt.z = 0.03;
    marker.points.push_back(pt);
  }

  points_->markers.push_back(std::move(marker));
}

std_msgs::msg::ColorRGBA TrajectoryVisualizer::costToColor(float normalized)
{
  // Green (0) -> Yellow (0.5) -> Red (1.0)
  normalized = std::clamp(normalized, 0.0f, 1.0f);
  float r, g;
  if (normalized < 0.5f) {
    r = 2.0f * normalized;
    g = 1.0f;
  } else {
    r = 1.0f;
    g = 2.0f * (1.0f - normalized);
  }
  return utils::createColor(r, g, 0.0f, 0.8f);
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
  optimal_path_ = std::make_unique<nav_msgs::msg::Path>();
}

void TrajectoryVisualizer::visualize()
{
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  if (optimal_path_pub_->get_subscription_count() > 0) {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }

  reset();
}

}  // namespace mppi
