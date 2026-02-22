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
    utils::createTrajectoryMarkers(trajectories, total_costs, "Total Costs", cmd_stamp,
      frame_id_, trajectory_step_, time_step_, marker_id_, *points_);
  }

  // Visualize each critic's contribution if requested
  if (visualize_per_critic) {
    for (const auto & [critic_name, costs] : individual_critics_cost) {
      utils::createTrajectoryMarkers(trajectories, costs, critic_name, cmd_stamp,
        frame_id_, trajectory_step_, time_step_, marker_id_, *points_);
    }
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
  if (publish_optimal_path_ && optimal_trajectory.rows() > 0) {
    add(optimal_trajectory, "Optimal Trajectory", stamp);
  }

  // Publish trajectories
  if (trajectories_publisher_ && trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  // Publish optimal path if enabled
  if (publish_optimal_path_ && optimal_path_pub_ &&
    optimal_path_pub_->get_subscription_count() > 0)
  {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }

  // Publish optimal footprints if enabled
  if (publish_optimal_footprints_ && optimal_footprints_pub_ &&
    optimal_footprints_pub_->get_subscription_count() > 0 &&
    costmap_ros != nullptr && optimal_trajectory.rows() > 0)
  {
    auto footprints_msg = utils::createFootprintMarkers(
      optimal_trajectory, header, costmap_ros, footprint_downsample_factor_);
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

  if (publish_optimal_path_ && optimal_path_pub_ &&
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

}  // namespace mppi
