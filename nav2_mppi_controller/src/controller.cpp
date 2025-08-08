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

#include <stdint.h>
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

void MPPIController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent, name_);

  auto node = parent_.lock();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);

  getParam(publish_optimal_trajectory_, "publish_optimal_trajectory", false);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  if (publish_optimal_trajectory_) {
    opt_traj_pub_ = node->create_publisher<nav2_msgs::msg::Trajectory>(
      "~/optimal_trajectory");
  }

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  opt_traj_pub_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  auto node = parent_.lock();
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  if (opt_traj_pub_) {
    opt_traj_pub_->on_activate();
  }
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  if (opt_traj_pub_) {
    opt_traj_pub_->on_deactivate();
  }
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset(false /*Don't reset zone-based speed limits between requests*/);
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  geometry_msgs::msg::Pose goal = path_handler_.getTransformedGoal(robot_pose.header.stamp).pose;

  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  auto [cmd, optimal_trajectory] =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (publish_optimal_trajectory_ && opt_traj_pub_->get_subscription_count() > 0) {
    auto trajectory_msg = utils::toTrajectoryMsg(
      optimal_trajectory,
      optimizer_.getOptimalControlSequence(),
      optimizer_.getSettings().model_dt,
      cmd.header);
    opt_traj_pub_->publish(std::move(trajectory_msg));
  }

  if (visualize_) {
    visualize(std::move(transformed_plan), cmd.header.stamp, optimal_trajectory);
  }

  return cmd;
}

void MPPIController::visualize(
  nav_msgs::msg::Path transformed_plan,
  const builtin_interfaces::msg::Time & cmd_stamp,
  const Eigen::ArrayXXf & optimal_trajectory)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimal_trajectory, "Optimal Trajectory", cmd_stamp);
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(
  const nav_msgs::msg::Path & path,
  const std::vector<geometry_msgs::msg::PoseStamped> & /*key_poses*/)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
