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
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParentParam(transform_tolerance_, "transform_tolerance", 0.1, ParameterType::Static);
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);

  getParam(publish_optimal_trajectory_, "publish_optimal_trajectory", false);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
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
  nav2_core::GoalChecker * goal_checker,
  nav_msgs::msg::Path & pruned_global_plan)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  unsigned int mx, my;
  // Find the furthest relevant pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  for (auto global_plan_pose = pruned_global_plan.poses.begin();
    global_plan_pose != pruned_global_plan.poses.end();
    ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = robot_pose.header.stamp;
    global_plan_pose->header.frame_id = pruned_global_plan.header.frame_id;
    nav2_util::transformPoseInTargetFrame(*global_plan_pose, costmap_plan_pose, *tf_buffer_,
        costmap_ros_->getGlobalFrameID(), transform_tolerance_);

    // Check if pose is inside the costmap
    if (!costmap_ros_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      break;
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  //TODO: add goal here
  geometry_msgs::msg::Pose goal;

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
    std_msgs::msg::Header trajectory_header;
    trajectory_header.stamp = cmd.header.stamp;
    trajectory_header.frame_id = costmap_ros_->getGlobalFrameID();

    auto trajectory_msg = utils::toTrajectoryMsg(
      optimal_trajectory,
      optimizer_.getOptimalControlSequence(),
      optimizer_.getSettings().model_dt,
      trajectory_header);
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

void MPPIController::newPathReceived(const nav_msgs::msg::Path & /*raw_global_path*/)
{
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
