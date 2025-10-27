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

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  auto node = parent_.lock();
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
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
  auto start = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  geometry_msgs::msg::Pose goal = path_handler_.getTransformedGoal(robot_pose.header.stamp).pose;

  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  auto [cmd, optimal_trajectory] =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal, goal_checker);

  auto computation_end = std::chrono::steady_clock::now();
  auto computation_time = std::chrono::duration_cast<std::chrono::microseconds>(
    computation_end - start).count();

  // Visualize everything in one consolidated call
  trajectory_visualizer_.visualize(
    std::move(transformed_plan),
    optimal_trajectory,
    optimizer_.getOptimalControlSequence(),
    optimizer_.getSettings().model_dt,
    cmd.header.stamp,
    costmap_ros_,
    optimizer_.getGeneratedTrajectories(),
    optimizer_.getCosts(),
    optimizer_.getCriticCosts());

  auto visualization_end = std::chrono::steady_clock::now();
  auto visualization_time = std::chrono::duration_cast<std::chrono::microseconds>(
    visualization_end - computation_end).count();
  auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(
    visualization_end - start).count();

  // Throttled info message every 5 seconds
  RCLCPP_INFO_THROTTLE(
    logger_, *parent_.lock()->get_clock(), 1000,
    "Control loop timing - Computation: %ld μs, Visualization: %ld μs (%.1f%% of total %ld μs)",
    computation_time, visualization_time,
    (total_time > 0 ? (100.0 * visualization_time / total_time) : 0.0),
    total_time);

  return cmd;
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
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
