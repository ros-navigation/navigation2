// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_graceful_motion_controller/graceful_motion_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_graceful_motion_controller
{

void GracefulMotionController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  parent_node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_,
    costmap_ros_->getCostmap()->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_buffer_, costmap_ros_);

  // Handles the control law to generate the velocity commands
  control_law_ = std::make_unique<SmoothControlLaw>(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda, params_->slowdown_radius,
    params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  // TODO(ajtudela): Add collision object

  // Publishers
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  motion_target_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("motion_target", 1);
  slowdown_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("slowdown", 1);

  RCLCPP_INFO(logger_, "Configured Graceful Motion Controller: %s", plugin_name_.c_str());
}

void GracefulMotionController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type graceful_motion_controller::GracefulMotionController",
    plugin_name_.c_str());
  transformed_plan_pub_.reset();
  local_plan_pub_.reset();
  motion_target_pub_.reset();
  slowdown_pub_.reset();
}

void GracefulMotionController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type graceful_motion_controller::GracefulMotionController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  motion_target_pub_->on_activate();
  slowdown_pub_->on_activate();
}

void GracefulMotionController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type graceful_motion_controller::GracefulMotionController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  motion_target_pub_->on_deactivate();
  slowdown_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped GracefulMotionController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tolerance_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame and publish it
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist);
  transformed_plan_pub_->publish(transformed_plan);

  // Get the particular point on the path at the motion target distance and publish it
  auto motion_target = getMotionTarget(params_->motion_target_dist, transformed_plan);
  auto motion_target_point = createMotionTargetMsg(motion_target);
  motion_target_pub_->publish(motion_target_point);

  // Publish marker for slowdown radius around motion target for debugging / visualization
  auto slowdown_marker = createSlowdownMsg(motion_target);
  slowdown_pub_->publish(slowdown_marker);

  // Compute velocity command
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist = control_law_->calculateRegularVelocity(motion_target.pose);

  // TODO(ajtudela): Check collision

  // Generate and publish local plan for debugging / visualization
  nav_msgs::msg::Path local_plan = simulateTrajectory(motion_target);
  local_plan.header = transformed_plan.header;
  local_plan_pub_->publish(local_plan);

  return cmd_vel;
}

void GracefulMotionController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
}

void GracefulMotionController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  // TODO(ajtudela): Add angular?
  // FIXME: Check this
  double max_linear_vel_ = 0.5;
  double base_linear_vel_ = 0.5;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    max_linear_vel_ = base_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      max_linear_vel_ = base_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in m/s
      max_linear_vel_ = speed_limit;
    }
  }
  base_linear_vel_ = max_linear_vel_;
}

geometry_msgs::msg::PoseStamped GracefulMotionController::getMotionTarget(
  const double & motion_target_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the motion target distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= motion_target_dist;
    });

  // If the pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  return *goal_pose_it;
}

geometry_msgs::msg::PointStamped GracefulMotionController::createMotionTargetMsg(
  const geometry_msgs::msg::PoseStamped & motion_target)
{
  geometry_msgs::msg::PointStamped motion_target_point;
  motion_target_point.header = motion_target.header;
  motion_target_point.point = motion_target.pose.position;
  motion_target_point.point.z = 0.01;
  return motion_target_point;
}

visualization_msgs::msg::Marker GracefulMotionController::createSlowdownMsg(
  const geometry_msgs::msg::PoseStamped & motion_target)
{
  visualization_msgs::msg::Marker slowdown_marker;
  slowdown_marker.header = motion_target.header;
  slowdown_marker.ns = "slowdown";
  slowdown_marker.id = 0;
  slowdown_marker.type = visualization_msgs::msg::Marker::SPHERE;
  slowdown_marker.action = visualization_msgs::msg::Marker::ADD;
  slowdown_marker.pose = motion_target.pose;
  slowdown_marker.pose.position.z = 0.01;
  slowdown_marker.scale.x = params_->slowdown_radius * 2.0;
  slowdown_marker.scale.y = params_->slowdown_radius * 2.0;
  slowdown_marker.scale.z = 0.02;
  slowdown_marker.color.a = 0.2;
  slowdown_marker.color.r = 0.0;
  slowdown_marker.color.g = 1.0;
  slowdown_marker.color.b = 0.0;
  return slowdown_marker;
}

nav_msgs::msg::Path GracefulMotionController::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & motion_target)
{
  nav_msgs::msg::Path trajectory;
  // First pose
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  next_pose.pose.orientation.w = 1.0;
  trajectory.poses.push_back(next_pose);

  double distance = std::numeric_limits<double>::max();
  double resolution_ = costmap_ros_->getCostmap()->getResolution();
  double dt = (params_->v_linear_max > 0.0) ? resolution_ / params_->v_linear_max : 0.0;
  // Set max iter to avoid infinite loop
  unsigned int max_iter = 2 * sqrt(motion_target.pose.position.x * motion_target.pose.position.x +
    motion_target.pose.position.y * motion_target.pose.position.y) / resolution_;
  // Generate path
  do{
    next_pose.pose = control_law_->calculateNextPose(dt, motion_target.pose, next_pose.pose);
    trajectory.poses.push_back(next_pose);
    double error_x = motion_target.pose.position.x - next_pose.pose.position.x;
    double error_y = motion_target.pose.position.y - next_pose.pose.position.y;
    distance = std::hypot(error_x, error_y);
  }while(distance > resolution_ && trajectory.poses.size() < max_iter);

  return trajectory;
}

}  // namespace nav2_graceful_motion_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_graceful_motion_controller::GracefulMotionController,
  nav2_core::Controller)
