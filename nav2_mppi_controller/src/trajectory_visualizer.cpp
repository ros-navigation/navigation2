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
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"

namespace mppi
{

void TrajectoryVisualizer::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  const std::string & frame_id, ParametersHandler * parameters_handler)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  frame_id_ = frame_id;
  trajectories_publisher_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectories", 1);
  transformed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  optimal_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/local_plan", 1);
  parameters_handler_ = parameters_handler;

  auto getParam = parameters_handler->getParamGetter(name + ".TrajectoryVisualizer");

  getParam(trajectory_step_, "trajectory_step", 5);
  getParam(time_step_, "time_step", 3);

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
  const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace)
{
  auto & size = trajectory.shape()[0];
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
    };

  for (size_t i = 0; i < size; i++) {
    add_marker(i);
  }

  optimal_traj_ = std::make_unique<xt::xtensor<float, 2>>(trajectory);
}

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories, const std::string & marker_namespace)
{
  auto & shape = trajectories.x.shape();
  const float shape_1 = static_cast<float>(shape[1]);
  points_->markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

  for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
    for (size_t j = 0; j < shape[1]; j += time_step_) {
      const float j_flt = static_cast<float>(j);
      float blue_component = 1.0f - j_flt / shape_1;
      float green_component = j_flt / shape_1;

      auto pose = utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
      auto scale = utils::createScale(0.03, 0.03, 0.03);
      auto color = utils::createColor(0, green_component, blue_component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);

      points_->markers.push_back(marker);
    }
  }
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = std::make_unique<visualization_msgs::msg::MarkerArray>();
  optimal_traj_ = std::make_unique<xt::xtensor<float, 2>>();
}

void TrajectoryVisualizer::visualize(
  const nav_msgs::msg::Path & plan,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }

  if (optimal_path_pub_->get_subscription_count() > 0) {
    publish_optimal_path(cmd_stamp);
  }

  reset();

  if (transformed_path_pub_->get_subscription_count() > 0) {
    auto plan_ptr = std::make_unique<nav_msgs::msg::Path>(plan);
    transformed_path_pub_->publish(std::move(plan_ptr));
  }
}

void TrajectoryVisualizer::publish_optimal_path(const builtin_interfaces::msg::Time & cmd_stamp)
{
  auto & size = optimal_traj_->shape()[0];
  if (size == 0) {
    return;
  }

  nav_msgs::msg::Path optimal_path;
  optimal_path.header.stamp = cmd_stamp;
  optimal_path.header.frame_id = frame_id_;

  for (size_t i = 0; i < size; i++) {
    // create new pose for the path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id_;

    // position & orientation
    pose_stamped.pose = utils::createPose((*optimal_traj_)(i, 0), (*optimal_traj_)(i, 1), 0.0);

    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(0., 0., (*optimal_traj_)(i, 2));
    pose_stamped.pose.orientation = tf2::toMsg(quaternion_tf2);

    // add pose to the path
    optimal_path.poses.push_back(pose_stamped);
  }
  optimal_path_pub_->publish(optimal_path);
}

}  // namespace mppi
